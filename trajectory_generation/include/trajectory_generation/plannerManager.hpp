/**
 * @file plannerManager.hpp
 * @brief 规划管理器 - 协调各规划模块的核心调度器
 */

#ifndef PLANNER_MANAGER_HPP
#define PLANNER_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include "trajectory_generation/planner_config.hpp"
#include "trajectory_generation/GridMap.hpp"
#include "trajectory_generation/Astar_searcher.hpp"
#include "trajectory_generation/TopoSearch.hpp"
#include "trajectory_generation/path_smooth.hpp"
#include "trajectory_generation/reference_path.hpp"

/**
 * @brief 规划管理器命名空间
 *
 * 功能:
 * - 协调各个规划模块(TopoSearch, A*, Smoother, ReferencePath)
 * - 执行完整的路径规划流程
 * - 管理全局和局部规划结果
 *
 * 规划流程:
 * 1. TopoSearcher::createGraph() - 构建PRM图
 * 2. AstarPathFinder::smoothTopoPath() - 路径剪枝
 * 3. Smoother::smoothPath() - L-BFGS优化
 * 4. Refenecesmooth::setGlobalPath() - 生成轨迹
 */
namespace planner_manager
{
    // ==================== 规划模块 ====================
    std::unique_ptr<AstarPathFinder> astar_path_finder;  // A*路径搜索器
    std::shared_ptr<GlobalMap> global_map;               // 全局地图
    std::unique_ptr<Smoother> path_smoother;             // 路径平滑器
    std::unique_ptr<Refenecesmooth> reference_path;      // 参考轨迹生成器
    std::unique_ptr<TopoSearcher> topo_prm;              // 拓扑搜索器

    // ==================== 规划结果 ====================
    std::vector<Eigen::Vector3d> optimized_path;      // 全局优化路径
    std::vector<Eigen::Vector3d> local_optimize_path; // 局部优化路径
    std::vector<Eigen::Vector3d> ref_trajectory;      // 参考轨迹(用于可视化和重规划)
    std::vector<Eigen::Vector3d> astar_path;          // A*搜索路径
    std::vector<Eigen::Vector2d> final_path;          // 最终路径
    std::vector<Eigen::Vector2d> final_path_temp;     // 临时路径(可���化用)
    std::vector<GraphNode::Ptr> global_graph;         // 全局PRM图

    // ==================== 动力学参数 ====================
    double reference_v_max;           // 最大速度
    double reference_a_max;           // 最大加速度
    double reference_w_max;           // 最大角速度
    double reference_desire_speed;    // 期望速度(正常模式)
    double reference_desire_speedxtl; // 期望速度(小陀螺模式)

    // ==================== 状态标志 ====================
    bool isxtl;     // 电控陀螺标志位
    bool xtl_flag;  // 规划陀螺标志位

    // ==================== 随机采样器 ====================
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;

    // ==================== 队伍颜色 ====================
    typedef enum {
        red = 0,
        blue,
    } teamColor;
    teamColor sentryColor;

    // ==================== 公共接口 ====================
    /**
     * @brief 初始化规划管理器
     * @param node ROS2节点指针
     */
    void init(rclcpp::Node::SharedPtr node);

    /**
     * @brief 全局路径规划
     * @param start_pt 起点
     * @param target_pt 目标点
     * @param start_vel 起始速度
     * @return true=规划成功
     */
    bool pathFinding(const Eigen::Vector3d start_pt,
                     const Eigen::Vector3d target_pt,
                     const Eigen::Vector3d start_vel);

    /**
     * @brief 重规划
     * @param start_pt 起点
     * @param target_pt 目标点
     * @param start_vel 起始速度
     * @return true=规划成功
     */
    bool replanFinding(const Eigen::Vector3d start_pt,
                       const Eigen::Vector3d target_pt,
                       const Eigen::Vector3d start_vel);

    /**
     * @brief 局部路径规划
     * @param start_pt 起点
     * @param target_pt 目标点
     * @return 局部路径
     */
    std::vector<Eigen::Vector3d> localPathFinding(const Eigen::Vector3d start_pt,
                                                   const Eigen::Vector3d target_pt);

    /**
     * @brief 发布全局规划结果(可视化)
     */
    void pubGlobalPlanningResult(std::vector<Eigen::Vector3d> nodes);
}

#endif
