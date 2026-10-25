cmake_minimum_required(VERSION 3.20)
project(robot_mobile_bringup)

find_package(ament_cmake REQUIRED)

# Configuration seulement : les outils Python sont dans robot_mobile_evaluation.
# scripts/generate_textures.py est un outil de développement, il n'est pas installé.
install(
  DIRECTORY launch config rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
