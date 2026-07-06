cmake_minimum_required(VERSION 3.16)

project(mujoco_try LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/mujoco/mujoco-3.10.0"
    CACHE PATH "Path to the MuJoCo SDK")

if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo header not found: ${MUJOCO_ROOT}/include/mujoco/mujoco.h")
endif()

if(NOT EXISTS "${MUJOCO_ROOT}/lib/libmujoco.so")
  message(FATAL_ERROR "MuJoCo library not found: ${MUJOCO_ROOT}/lib/libmujoco.so")
endif()

add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco
  PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_ROOT}/lib/libmujoco.so"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include;${MUJOCO_ROOT}/include/mujoco")

add_subdirectory(libs/hello)
add_subdirectory(tutorial)
