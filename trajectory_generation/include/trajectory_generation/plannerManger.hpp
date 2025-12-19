#include <rclcpp/rclcpp.hpp>
#include "trajectory_generation/planner_config.hpp"
#include "trajectory_generation/GridMap.hpp"
#include "trajectory_generation/Astar_searcher.hpp"
#include "trajectory_generation/TopoSearch.hpp"
#include "trajectory_generation/path_smooth.hpp"
#include "trajectory_generation/reference_path.hpp"

namespace planner_manger
{
    std::unique_ptr<AstarPathFinder> astar_path_finder;
    std::shared_ptr<GlobalMap> global_map;
    std::unique_ptr<Smoother> path_smoother;
    std::unique_ptr<Refenecesmooth> reference_path;
    std::unique_ptr<TopoSearcher> topo_prm;

    /*全局规划结果与局部规划结果*/
    std::vector<Eigen::Vector3d> optimized_path;
    std::vector<Eigen::Vector3d> local_optimize_path;
    std::vector<Eigen::Vector3d> ref_trajectory;  // 可视化和重规划判断
    std::vector<Eigen::Vector3d> astar_path;
    std::vector<Eigen::Vector2d> final_path;
    std::vector<Eigen::Vector2d> final_path_temp;  // 临时可视化变量
    std::vector<GraphNode::Ptr> global_graph;

    /// 参考路径的最大线avw
    double reference_v_max;
    double reference_a_max;
    double reference_w_max;
    double reference_desire_speed;
    double reference_desire_speedxtl;

    /* 相关标志位 */
    bool isxtl;    // 电控陀螺标志位
    bool xtl_flag; // 规划陀螺标志位


    // 初始化函数
    void init(rclcpp::Node::SharedPtr node);
    std::vector<Eigen::Vector3d> localPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);
    bool pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                     const Eigen::Vector3d start_vel);
    bool replanFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                       const Eigen::Vector3d start_vel);
    void pubGlobalPlanningResult(std::vector<Eigen::Vector3d> nodes);
}