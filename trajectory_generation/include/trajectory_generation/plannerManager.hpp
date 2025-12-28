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
    extern std::unique_ptr<AstarPathFinder> astar_path_finder;
    extern std::shared_ptr<GlobalMap> global_map;
    extern std::unique_ptr<Smoother> path_smoother;
    extern std::unique_ptr<Refenecesmooth> reference_path;
    extern std::unique_ptr<TopoSearcher> topo_prm;

    // ==================== 规划结果 ====================
    extern std::vector<Eigen::Vector3d> optimized_path;
    extern std::vector<Eigen::Vector3d> local_optimize_path;
    extern std::vector<Eigen::Vector3d> ref_trajectory;
    extern std::vector<Eigen::Vector3d> astar_path;
    extern std::vector<Eigen::Vector2d> final_path;
    extern std::vector<Eigen::Vector2d> final_path_temp;
    extern std::vector<GraphNode::Ptr> global_graph;

    // ==================== 动力学参数 ====================
    extern double reference_v_max;
    extern double reference_a_max;
    extern double reference_w_max;
    extern double reference_desire_speed;
    extern double reference_desire_speedxtl;

    // ==================== 状态标志 ====================
    extern bool isxtl;
    extern bool xtl_flag;

    // ==================== 随机采样器 ====================
    extern std::random_device m_rd;
    extern std::default_random_engine m_eng;
    extern std::uniform_real_distribution<double> m_rand_pos;

    // ==================== 队伍颜色 ====================
    typedef enum { red = 0, blue } teamColor;
    extern teamColor sentryColor;

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
