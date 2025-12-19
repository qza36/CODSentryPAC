#include "trajectory_generation/plannerManger.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("trajectory_generation.planner_manager");
}

bool planner_manger::pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt, const Eigen::Vector3d start_vel) {
    RCLCPP_WARN(kLogger, "[Manager] start point, (x, y): (%.2f, %.2f)", start_pt.x(), start_pt.y());
    RCLCPP_WARN(kLogger, "[Manager] receive target, (x, y): (%.2f, %.2f)", target_pt.x(), target_pt.y());
    topo_prm->createGraph(start_pt, target_pt);

    optimized_path.clear();
    std::vector<Eigen::Vector3d> origin_path;
    if(topo_prm->min_path.size() > 0){
        origin_path = topo_prm->min_path;
    }else{
        RCLCPP_ERROR(kLogger, "[Manager PLANNING] Invalid target point，global planning failed");
        global_map->resetUsedGrids();
        return false;
    }
    /* 提取出路径（将路径节点都放到一个容器里） */

    /* 二次规划（路径裁减） */
    optimized_path = astar_path_finder->smoothTopoPath(origin_path);  // 剪枝优化topo路径
    std::cout<<"optimized_path size: "<<optimized_path.size()<<std::endl;
    global_map->resetUsedGrids();

    rclcpp::Time t1,t2;
    t1 = rclcpp::Clock().now();
    double reference_speed = isxtl? reference_desire_speedxtl : reference_desire_speed;


    RCLCPP_WARN(kLogger, "[Manager] reference_speed: (%.2f)", reference_speed);

    path_smoother->init(optimized_path, start_vel, reference_speed);
    path_smoother->smoothPath();
    path_smoother->pathResample();
    final_path = path_smoother->getPath();
    RCLCPP_INFO(kLogger, "[Manager] optimizer generate time: %f", (rclcpp::Clock().now() - t1).seconds());

    reference_path->setGlobalPath(start_vel, final_path, reference_a_max, reference_speed, isxtl);
    reference_path->getRefTrajectory(ref_trajectory, path_smoother->m_trapezoidal_time);
    reference_path->getRefVel();

    astar_path = origin_path;
    if(ref_trajectory.size() < 2){
        return false;
    }
    return true;
}
