cmake_minimum_required(VERSION 3.8)
project(xarm_moveit_planning_scene)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  yasmin
  yasmin_ros
  moveit_visual_tools
  moveit_ros_planning_interface
)


# planning_scene
add_executable(planning_scene src/planning_scene.cpp)
ament_target_dependencies(planning_scene ${DEPENDENCIES})
install(TARGETS 
  planning_scene
  DESTINATION lib/${PROJECT_NAME}
)

# publish_scene
add_executable(publish_scene src/publish_scene.cpp)
ament_target_dependencies(publish_scene ${DEPENDENCIES})
install(TARGETS 
  publish_scene
  DESTINATION lib/${PROJECT_NAME}
)

# scene_manager_state
add_executable(scene_manager_state src/scene_manager_state.cpp src/scene_manager.cpp)
ament_target_dependencies(scene_manager_state ${DEPENDENCIES})
install(TARGETS 
  scene_manager_state
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
