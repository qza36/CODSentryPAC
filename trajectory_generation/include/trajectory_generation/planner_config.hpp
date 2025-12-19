#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP
#include <rclcpp/rclcpp.hpp>
#include  "Eigen/Core"
#include <iostream>
#include <string>
namespace planner_manager
{
    struct PlannerConfig
    {
        //地图参数
        struct MapConfig
        {
            std::string occ_map_path;
            std::string bev_map_path;
            std::string distance_map_path;
            std::string topo_map_path;
            double height_bias;
            double height_interval;
            double height_threshold;
            double height_sencond_high_threshold; //狗洞
            double map_resolution;
            Eigen::Vector3d map_lower_point;
            Eigen::Vector3d map_upper_point;
            Eigen::Vector3i map_grid_size;
        }map;
        //搜索参数
        struct PathSearchConfig
        {
            double search_height_min;
            double search_height_max;
            double search_radius;
            double robot_radius;
            double robot_radius_dash;
        }search;
    };
}
#endif

