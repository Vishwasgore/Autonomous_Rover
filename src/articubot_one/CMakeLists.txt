cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==================== PYTHON SCRIPT INSTALLATION ====================
# Create scripts directory if it doesn't exist
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

# Install all Python scripts as executables
install(PROGRAMS
  scripts/mpu6050_driver.py
  scripts/motor_controller.py
  scripts/test_gpio.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, and other resources
install(
  DIRECTORY 
    config 
    description 
    launch 
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the entire scripts directory for development
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)
# ====================================================================

ament_package()