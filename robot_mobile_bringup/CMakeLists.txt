cmake_minimum_required(VERSION 3.20)
project(robot_mobile_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Outils d'acquisition, lancés avec : ros2 run robot_mobile_bringup <outil>.py
# (generate_textures.py est un outil de développement, il n'est pas installé)
install(
  PROGRAMS
    scripts/reference_trajectory.py
    scripts/sensor_check.py
    scripts/analyze_dataset.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
