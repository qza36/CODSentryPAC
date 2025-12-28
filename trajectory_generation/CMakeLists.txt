cmake_minimum_required(VERSION 3.8)
project(trajectory_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# msgs
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoly.msg"
  DEPENDENCIES builtin_interfaces
)

add_executable(trajectory_generation_node src/trajectory_generator_node.cpp
        src/Astar_searcher.cpp
        src/GridMap.cpp
        src/planner_config.cpp
        src/TopoSearch.cpp
        src/plannerManager.cpp
        src/reference_path.cpp
        src/path_smooth.cpp
        src/visualization_utils.cpp
        src/replan_fsm.cpp
)

add_executable(fake_odom_node src/fake_odom_node.cpp)

ament_target_dependencies(trajectory_generation_node
        std_msgs
        sensor_msgs
        PCL
        rclcpp
        OpenCV
        nav_msgs
        gazebo_msgs
        pcl_conversions
        geometry_msgs
        visualization_msgs
)
target_include_directories(trajectory_generation_node
        PRIVATE
        include
        utils
)
target_link_libraries(trajectory_generation_node
        ${OpenCV_LIBRARIES}
)

# 链接生成的消息
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(trajectory_generation_node "${cpp_typesupport_target}")

ament_target_dependencies(fake_odom_node rclcpp nav_msgs)

install(TARGETS
        trajectory_generation_node
        fake_odom_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
