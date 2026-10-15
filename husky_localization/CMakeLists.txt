cmake_minimum_required(VERSION 3.8)
project(husky_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Required to install a Python module from an ament_cmake package.
# Without it, ament_python_install_package() is not defined.
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------------------------------------------------------------------
# Python module -> installed so that "from husky_localization.x import y" works
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Executables
#
# These are thin wrappers that import from the module above.
# They MUST go into lib/${PROJECT_NAME} -- that is the only directory
# `ros2 run` searches. Installing them to bin/ or share/ will build fine
# and then fail at runtime with "No executable found".
#
# The files have no .py extension on purpose, so the command stays
# `ros2 run husky_localization gps_sim_node` rather than `... gps_sim_node.py`.
# ---------------------------------------------------------------------------
install(PROGRAMS
  scripts/gps_sim_node
  scripts/odom_drift_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Launch files and parameter files
#
# get_package_share_directory() resolves to share/${PROJECT_NAME}, so anything
# the launch file reads at runtime has to be installed there.
# ---------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
