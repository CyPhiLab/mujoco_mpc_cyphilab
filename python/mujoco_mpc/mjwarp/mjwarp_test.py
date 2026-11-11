# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Tests for MJWarp trajectory optimization (run on Warp's CPU device too).

  python -m pytest python/mujoco_mpc/mjwarp/mjwarp_test.py
"""

from absl.testing import absltest
import mujoco
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import cem as cem_lib
from mujoco_mpc.mjwarp import derivatives as deriv_lib
from mujoco_mpc.mjwarp import ilqr as ilqr_lib
from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import norms
from mujoco_mpc.mjwarp import rollout as rollout_lib
from mujoco_mpc.mjwarp import state as state_lib
from mujoco_mpc.mjwarp.tasks import base

wp.config.quiet = True

# pendulum (hinge) + a ball-jointed rod driven by two motors: no contacts.
_XML = """
<mujoco>
  <option timestep="0.01"/>
  <worldbody>
    <body name="pole" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.04" mass="1"/>
    </body>
    <body name="rod" pos="1 0 1">
      <joint name="ball" type="ball" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 -0.3" size="0.03" mass="0.5"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" gear="3" ctrlrange="-1 1" ctrllimited="true"/>
    <general joint="ball" gear="0 1 0 0 0 0" ctrlrange="-1 1" ctrllimited="true"/>
  </actuator>
  <sensor>
    <user name="Angle" dim="1" user="0 1 0 1"/>
    <user name="Velocity" dim="1" user="6 0.1 0 1 0.3"/>
    <user name="Control" dim="2" user="2 0.05 0 1 0.1"/>
  </sensor>
</mujoco>
"""


@wp.kernel
def _swingup_kernel(
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    ctrl: wp.array2d(dtype=float),
    stage_time: wp.array(dtype=float),
    residual: wp.array2d(dtype=float),
    weight: wp.array2d(dtype=float),
):
  w = wp.tid()
  # hinge angle to upright (pi), shortest way
  e = qpos[w, 0] - wp.pi
  residual[w, 0] = wp.atan2(wp.sin(e), wp.cos(e))
  residual[w, 1] = qvel[w, 0]
  residual[w, 2] = ctrl[w, 0]
  residual[w, 3] = ctrl[w, 1]
  # time-varying weight: angle term only counts in the second half
  weight[w, 0] = wp.where(stage_time[w] > 0.5, 1.0, 0.0)
  weight[w, 1] = 0.1
  weight[w, 2] = 0.05


class SwingUp(base.Task):

  def residual(self, m, d, stage_time, residual, weight):
    wp.launch(
        _swingup_kernel,
        dim=d.nworld,
        inputs=[d.qpos, d.qvel, d.ctrl, stage_time],
        outputs=[residual, weight],
    )


def _model():
  return mujoco.MjModel.from_xml_string(_XML)


def _x0(mjm):
  d = mujoco.MjData(mjm)
  d.qpos[0] = 0.3
  d.qpos[1:5] = [np.cos(0.2), np.sin(0.2), 0, 0]
  d.qvel[:] = [0.1, 0.2, -0.1, 0.3]
  return np.concatenate([d.qpos, d.qvel])


class MJWarpTest(absltest.TestCase):

  def test_cost_spec(self):
    spec = model_lib.cost_spec(_model())
    self.assertEqual(spec.names, ("Angle", "Velocity", "Control"))
    np.testing.assert_array_equal(spec.dims, [1, 1, 2])
    np.testing.assert_array_equal(spec.offsets, [0, 1, 2])
    np.testing.assert_array_equal(spec.norms, [0, 6, 2])
    np.testing.assert_allclose(spec.params[:, 0], [0, 0.3, 0.1])

  def test_norm_derivatives(self):
    spec = model_lib.cost_spec(_model())
    rng = np.random.default_rng(0)
    r = rng.normal(size=spec.nresidual)
    w = rng.uniform(0.5, 2, size=spec.nterm)
    c, g, H = norms.norm_derivatives(spec, r, w)
    eps = 1e-6
    for i in range(spec.nresidual):
      dr = np.zeros_like(r)
      dr[i] = eps
      cp, gp, _ = norms.norm_derivatives(spec, r + dr, w)
      cm, gm, _ = norms.norm_derivatives(spec, r - dr, w)
      self.assertAlmostEqual(g[i], (cp - cm) / (2 * eps), places=5)
      np.testing.assert_allclose(H[:, i], (gp - gm) / (2 * eps), atol=1e-5)
    # device cost matches host cost
    ds = norms.DeviceCostSpec(spec)
    total = wp.zeros(1, dtype=float)
    stage = wp.zeros((1, 1), dtype=float)
    wp.launch(
        norms.stage_cost_kernel, dim=1,
        inputs=[
            wp.array(r[None].astype(np.float32), dtype=float),
            wp.array(w[None].astype(np.float32), dtype=float),
            ds.dims, ds.offsets, ds.norms, ds.params, 1.0, 0,
        ],
        outputs=[total, stage],
    )
    self.assertAlmostEqual(float(total.numpy()[0]), c, places=4)

  def test_rollout_matches_cpu(self):
    mjm = _model()
    task = SwingUp(mjm)
    T, nworld = 50, 3
    x0 = _x0(mjm)
    r = rollout_lib.Rollout(mjm, task, nworld, T, use_graph=False)
    r.set_sources(x0[None])
    pol = rollout_lib.SplinePolicy(nworld, 5, mjm.nu, "cubic")
    knots = np.random.default_rng(1).uniform(-1, 1, (nworld, 5, mjm.nu))
    pol.set(knots)
    cost = r.run(pol)
    states, actions = r.get_states(), r.get_actions()
    stage = r.get_stage_costs()
    spec = task.spec
    for w in range(nworld):
      d = mujoco.MjData(mjm)
      d.qpos[:], d.qvel[:] = x0[: mjm.nq], x0[mjm.nq :]
      host_cost = 0.0
      for t in range(T):
        np.testing.assert_allclose(states[w, t, : mjm.nq], d.qpos, atol=2e-4)
        d.ctrl[:] = actions[w, t]
        e = d.qpos[0] - np.pi
        res = np.array([np.arctan2(np.sin(e), np.cos(e)), d.qvel[0], *d.ctrl])
        wt = np.array([1.0 if d.time > 0.5 else 0.0, 0.1, 0.05])
        c, _, _ = norms.norm_derivatives(spec, res, wt)
        self.assertAlmostEqual(stage[w, t], c, places=3)
        host_cost += c
        mujoco.mj_step(mjm, d)
      self.assertLess(abs(cost[w] - host_cost - stage[w, T]), 1e-2)

  def test_state_maps(self):
    mjm = _model()
    lay = state_lib.StateLayout(mjm)
    rng = np.random.default_rng(2)
    x1 = _x0(mjm)
    dx = rng.normal(scale=0.3, size=lay.ndx)
    x2 = lay.integrate_host(mjm, x1, dx)
    np.testing.assert_allclose(lay.difference_host(mjm, x1, x2), dx, atol=1e-10)

  def test_derivatives_match_mjd_transition_fd(self):
    mjm = _model()
    task = SwingUp(mjm)
    lay = state_lib.StateLayout(mjm)
    x0 = _x0(mjm)
    u = np.array([0.3, -0.2])
    fd = deriv_lib.FiniteDifference(mjm, task, max_worlds=64, eps=1e-3)
    T = 3
    states = np.zeros((1, T + 1, lay.nx))
    d = mujoco.MjData(mjm)
    d.qpos[:], d.qvel[:] = x0[: mjm.nq], x0[mjm.nq :]
    for t in range(T + 1):
      states[0, t] = np.concatenate([d.qpos, d.qvel])
      d.ctrl[:] = u
      mujoco.mj_step(mjm, d)
    actions = np.tile(u, (1, T, 1))
    times = np.arange(T + 1)[None] * mjm.opt.timestep
    der = fd.compute(states, actions, times)
    for t in range(T):
      d = mujoco.MjData(mjm)
      d.qpos[:], d.qvel[:] = states[0, t, : mjm.nq], states[0, t, mjm.nq :]
      d.ctrl[:] = u
      A = np.zeros((lay.ndx, lay.ndx))
      B = np.zeros((lay.ndx, mjm.nu))
      mujoco.mjd_transitionFD(mjm, d, 1e-6, 1, A, B, None, None)
      np.testing.assert_allclose(der.A[0, t], A, atol=5e-3)
      np.testing.assert_allclose(der.B[0, t], B, atol=5e-3)
    # residual Jacobian: d(ctrl residual)/du = I, d(velocity residual)/dqvel0
    np.testing.assert_allclose(der.Ju[0, 0, 2:, :], np.eye(2), atol=1e-3)
    self.assertAlmostEqual(der.Jx[0, 0, 1, mjm.nv + 0], 1.0, places=3)
    np.testing.assert_allclose(der.Ju[0, T], 0.0)

  def test_ilqr_reduces_cost(self):
    mjm = _model()
    task = SwingUp(mjm)
    x0 = _x0(mjm)
    T = 100
    opt = ilqr_lib.ILQR(
        mjm, task, T, x0, nproblem=2,
        config=ilqr_lib.ILQRConfig(iterations=8, max_fd_worlds=4096),
    )
    init = np.zeros((2, T, mjm.nu))
    init[1] = 0.5
    res = opt.optimize(init)
    self.assertTrue(np.all(np.diff(res.history, axis=0) <= 1e-6))
    self.assertTrue(np.all(res.history[-1] < 0.5 * res.history[0]))

  def test_cem_reduces_cost(self):
    mjm = _model()
    task = SwingUp(mjm)
    x0 = _x0(mjm)
    cfg = cem_lib.CEMConfig(nsample=32, nelite=6, nknot=6, iterations=6)
    opt = cem_lib.CEM(mjm, task, 100, x0, nproblem=2, config=cfg)
    res = opt.optimize()
    self.assertTrue(np.all(np.diff(res.history, axis=0) <= 1e-6))
    self.assertTrue(np.all(res.history[-1] < res.history[0]))
    self.assertEqual(res.states.shape, (2, 101, mjm.nq + mjm.nv))


if __name__ == "__main__":
  absltest.main()
