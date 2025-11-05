cmake_minimum_required(VERSION 3.14)
project(autoware_path_generator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(path_generator_parameters
  param/path_generator_parameters.yaml
)

# TODO: Remove this workaround when generate_parameter_library upstream removes
# 'using namespace parameter_traits;' from the template
# Reference: https://github.com/PickNikRobotics/generate_parameter_library/pull/91
# Current status: As of generate_parameter_library 0.5.0 (Humble/Jazzy), the template
# still includes 'using namespace parameter_traits;' for backwards compatibility,
# which triggers "parameter_traits::OK is deprecated" warnings.
# This workaround removes the deprecated namespace import and suppresses warnings
# from the parameter_traits.hpp system header.
# NOTE: Verified that this issue exists in both Humble and Jazzy (0.5.0).
# Check if this is still needed for Kilted or later distributions and remove if fixed.
set(GENERATED_HEADER "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/path_generator_parameters.hpp")
set(PATCH_STAMP "${CMAKE_CURRENT_BINARY_DIR}/path_generator_parameters.hpp.patched")

# Use a Python script for more reliable file patching
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/patch_header.py" "
import sys
with open(sys.argv[1], 'r') as f:
    content = f.read()
# Remove deprecated using statement
content = content.replace('using namespace parameter_traits;\\n', '')
content = content.replace('// temporarily needed for backwards compatibility\\n', '')
# Add pragma to suppress warnings from parameter_traits.hpp system header
content = content.replace(
    '#include <parameter_traits/parameter_traits.hpp>',
    '''#pragma GCC diagnostic push
#pragma GCC diagnostic ignored \\\"-Wdeprecated-declarations\\\"
#include <parameter_traits/parameter_traits.hpp>
#pragma GCC diagnostic pop'''
)
with open(sys.argv[1], 'w') as f:
    f.write(content)
# Write stamp file
with open(sys.argv[2], 'w') as f:
    f.write('patched')
")

# Create a custom command that depends on the generated header file
add_custom_command(
  OUTPUT "${PATCH_STAMP}"
  COMMAND python3 "${CMAKE_CURRENT_BINARY_DIR}/patch_header.py" "${GENERATED_HEADER}" "${PATCH_STAMP}"
  DEPENDS "${GENERATED_HEADER}"
  COMMENT "Removing deprecated parameter_traits usage from generated header"
  VERBATIM
)

# Create a target for the patch operation
add_custom_target(patch_parameter_header
  DEPENDS "${PATCH_STAMP}"
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# Ensure library is built after patching
add_dependencies(${PROJECT_NAME} patch_parameter_header)

target_link_libraries(${PROJECT_NAME}
  path_generator_parameters
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_generator::PathGenerator"
  EXECUTABLE path_generator_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_dense_centerline.cpp
    test/test_goal_connection.cpp
    test/test_path_generator_node_interface.cpp
    test/test_lanelet.cpp
    test/test_turn_signal.cpp
    test/test_path_cut.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    test_route
)
