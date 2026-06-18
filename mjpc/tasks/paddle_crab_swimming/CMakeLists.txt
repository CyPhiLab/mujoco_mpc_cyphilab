# CMakeLists.txt for PaddleCrabSwimming task

set(TASK_PADDLE_CRAB_SWIMMING_SRC
    paddle_crab_swimming.cc
    paddle_crab_swimming.h
)

add_library(paddle_crab_swimming STATIC ${TASK_PADDLE_CRAB_SWIMMING_SRC})
target_include_directories(paddle_crab_swimming PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(paddle_crab_swimming PUBLIC mjpc_core)
