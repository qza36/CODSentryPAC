#include <rclcpp/rclcpp.hpp>
#include "trajectory_generation/planner_config.hpp"
#include "trajectory_generation/GridMap.hpp"

namespace planner_manger
{
    // 这里持有 global_map 指针
    extern std::shared_ptr<GlobalMap> global_map;
    // 初始化函数
    void init(rclcpp::Node::SharedPtr node);
    std::vector<Eigen::Vector3d> localPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);
    bool pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                     const Eigen::Vector3d start_vel);
    bool replanFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                       const Eigen::Vector3d start_vel);
    void pubGlobalPlanningResult(std::vector<Eigen::Vector3d> nodes);
}