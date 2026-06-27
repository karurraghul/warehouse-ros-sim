cmake_minimum_required(VERSION 3.8)
project(robo_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(SHARE_DEST share/${PROJECT_NAME})

# Package resources
foreach(dir launch config gazebo worlds maps)
  install(DIRECTORY ${dir}
    DESTINATION ${SHARE_DEST}/
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
  )
endforeach()

# Robot assets used by URDF/xacro (package://robo_ai/models/...)
foreach(dir urdf meshes delivery_robot)
  install(DIRECTORY models/${dir}
    DESTINATION ${SHARE_DEST}/models
  )
endforeach()

# Gazebo warehouse models (folders with model.config)
file(GLOB MODEL_DIRS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/models"
  "${CMAKE_CURRENT_SOURCE_DIR}/models/*")

foreach(model_dir ${MODEL_DIRS})
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models/${model_dir}/model.config")
    install(DIRECTORY models/${model_dir}
      DESTINATION ${SHARE_DEST}/gazebo_models/models
    )
  endif()
endforeach()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/99-robo_ai-gazebo.dsv.in")

ament_package()