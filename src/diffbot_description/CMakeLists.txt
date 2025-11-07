cmake_minimum_required(VERSION 3.8)
project(diffbot_description)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# The critical step: tells CMake this is a Python package
find_package(ament_python REQUIRED)


ament_package()