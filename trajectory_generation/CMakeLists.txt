cmake_minimum_required(VERSION 3.8)
project(trajectory_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(trajectory_generation src/Astrar_searcher.cpp
        src/GridMap.cpp
        src/planner_config.cpp
)

ament_target_dependencies(trajectory_generation
        std_msgs
        sensor_msgs
        PCL
        rclcpp
        OpenCV
        nav_msgs
        gazebo_msgs
        sensor_msgs

)
target_include_directories(trajectory_generation
        PRIVATE
        include
        utils
)
target_link_libraries(trajectory_generation
        ${OpenCV_LIBRARIES}
)

install(TARGETS
        trajectory_generation
        DESTINATION lib/${PROJECT_NAME})
ament_package()
