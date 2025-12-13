#include <rclcpp/rclcpp.hpp>
#include "trajectory_generation/planner_config.hpp"
#include "trajectory_generation/GridMap.hpp"

namespace planner_manger
{
    // 这里持有 global_map 指针
    extern std::shared_ptr<GlobalMap> global_map;
    // 初始化函数
    void init(rclcpp::Node::SharedPtr node);
}