add_library(mujoco_tutorial_common
    src/common.cpp
    src/double_pendulum_controller.cpp)

target_include_directories(mujoco_tutorial_common
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(mujoco_tutorial_common
    PUBLIC
        mujoco::mujoco)

target_compile_features(mujoco_tutorial_common
    PUBLIC
        cxx_std_17)

target_compile_definitions(mujoco_tutorial_common
    PUBLIC
        TUTORIAL_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models"
        TUTORIAL_RM26_DIR="${CMAKE_CURRENT_SOURCE_DIR}/rm26_version3_engineering_model"
        TUTORIAL_MUJOCO_ROOT="${MUJOCO_ROOT}")

set_target_properties(mujoco_tutorial_common
    PROPERTIES
        BUILD_RPATH "${MUJOCO_ROOT}/lib")

function(add_mujoco_tutorial_demo target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE mujoco_tutorial_common)
    target_compile_features(${target} PRIVATE cxx_std_17)
    set_target_properties(${target}
        PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            BUILD_RPATH "${MUJOCO_ROOT}/lib")
endfunction()

add_mujoco_tutorial_demo(tutorial_01_load_and_step src/01_load_and_step.cpp)
add_mujoco_tutorial_demo(tutorial_02_state_access src/02_state_access.cpp)
add_mujoco_tutorial_demo(tutorial_03_control_loop src/03_control_loop.cpp)
add_mujoco_tutorial_demo(tutorial_04_step_split_controller src/04_step_split_controller.cpp)
add_mujoco_tutorial_demo(tutorial_05_callbacks src/05_callbacks.cpp)
add_mujoco_tutorial_demo(tutorial_06_dynamics_queries src/06_dynamics_queries.cpp)
add_mujoco_tutorial_demo(tutorial_07_model_editing src/07_model_editing.cpp)
add_mujoco_tutorial_demo(tutorial_08_visualization_scene src/08_visualization_scene.cpp)
add_mujoco_tutorial_demo(tutorial_09_sensor_suite src/09_sensor_suite.cpp)
add_mujoco_tutorial_demo(tutorial_10_double_inverted_pendulum src/10_double_inverted_pendulum.cpp)
add_mujoco_tutorial_demo(tutorial_11_rm26_import_check src/11_rm26_import_check.cpp)
add_mujoco_tutorial_demo(tutorial_12_ui_definition src/12_ui_definition.cpp)
add_mujoco_tutorial_demo(tutorial_13_plugin_loading src/13_plugin_loading.cpp)

find_package(OpenGL QUIET)
find_package(glfw3 QUIET)

set(MUJOCO_TUTORIAL_GLFW_TARGET "")
if(TARGET glfw)
    set(MUJOCO_TUTORIAL_GLFW_TARGET glfw)
elseif(TARGET glfw3)
    set(MUJOCO_TUTORIAL_GLFW_TARGET glfw3)
elseif(TARGET glfw3::glfw)
    set(MUJOCO_TUTORIAL_GLFW_TARGET glfw3::glfw)
endif()

if(OpenGL_FOUND AND MUJOCO_TUTORIAL_GLFW_TARGET)
    add_mujoco_tutorial_demo(tutorial_14_visualized_double_pendulum src/14_visualized_double_pendulum.cpp)
    target_link_libraries(tutorial_14_visualized_double_pendulum
        PRIVATE
            ${MUJOCO_TUTORIAL_GLFW_TARGET}
            OpenGL::GL)
else()
    message(WARNING
        "Skipping tutorial_14_visualized_double_pendulum because GLFW/OpenGL "
        "development packages were not found. Install packages discoverable as "
        "glfw3 and OpenGL to build the windowed visualization demo.")
endif()
