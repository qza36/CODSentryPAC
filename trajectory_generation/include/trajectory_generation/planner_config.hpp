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
            int topo_max_sample_num = 400;  // 拓扑搜索最大采样数量
        }search;
        //动力学参数
        struct DynamicsConfig
        {
            double v_max = 2.0;           // 最大速度 (m/s)
            double a_max = 3.0;           // 最大加速度 (m/s^2)
            double w_max = 2.0;           // 最大角速度 (rad/s)
            double desire_speed = 1.5;    // 期望速度 (m/s)
        }dynamics;
    };
}
#endif

