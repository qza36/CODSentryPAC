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

add_executable(trajectory_generation src/Astrar_searcher.cpp
        src/GridMap.cpp
)

ament_target_dependencies(trajectory_generation
        std_msgs
        sensor_msgs
        PCL
        rclcpp

)
target_include_directories(trajectory_generation
        PRIVATE
        include
        utils
)


ament_package()
