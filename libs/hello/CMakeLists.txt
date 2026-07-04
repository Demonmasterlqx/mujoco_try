add_executable(hello src/main.cpp)

target_link_libraries(hello PRIVATE mujoco::mujoco)
target_compile_features(hello PRIVATE cxx_std_17)

set_target_properties(hello
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    BUILD_RPATH "${MUJOCO_ROOT}/lib")

add_custom_command(
  TARGET hello
  POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "${CMAKE_CURRENT_SOURCE_DIR}/src/hello.xml"
          "$<TARGET_FILE_DIR:hello>/hello.xml")
