#include "trajectory_generation/plannerManager.hpp"
#include <rclcpp/rclcpp.hpp>

namespace planner_manager {
    // 变量定义
    std::unique_ptr<AstarPathFinder> astar_path_finder;
    std::shared_ptr<GlobalMap> global_map;
    std::unique_ptr<Smoother> path_smoother;
    std::unique_ptr<Refenecesmooth> reference_path;
    std::unique_ptr<TopoSearcher> topo_prm;

    std::vector<Eigen::Vector3d> optimized_path;
    std::vector<Eigen::Vector3d> local_optimize_path;
    std::vector<Eigen::Vector3d> ref_trajectory;
    std::vector<Eigen::Vector3d> astar_path;
    std::vector<Eigen::Vector2d> final_path;
    std::vector<Eigen::Vector2d> final_path_temp;
    std::vector<GraphNode::Ptr> global_graph;

    double reference_v_max;
    double reference_a_max;
    double reference_w_max;
    double reference_desire_speed;
    double reference_desire_speedxtl;

    bool isxtl;
    bool xtl_flag;

    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;

    teamColor sentryColor;
}

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("trajectory_generation.planner_manager");
}

bool planner_manager::pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt, const Eigen::Vector3d start_vel) {
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
std::vector<Eigen::Vector3d> planner_manager::localPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt)
{
    std::vector<Eigen::Vector3d> optimized_local_path;
    topo_prm->createLocalGraph(start_pt, target_pt);

    if(topo_prm->min_path.size() > 0){
        optimized_local_path = topo_prm->min_path;
        RCLCPP_DEBUG(kLogger,"[Manager Local] local path find! size is %d", topo_prm->min_path.size());
        return optimized_local_path;
    }else{
        RCLCPP_DEBUG(kLogger,"[Manager Local] Invalid target point，global planning failed");
        global_map->resetUsedGrids();
        return optimized_local_path;
    }
}
bool planner_manager::replanFinding(const Eigen::Vector3d start_point, const Eigen::Vector3d target_point,
                                    const Eigen::Vector3d start_vel)
{
    RCLCPP_INFO(kLogger, "[Manager REPLAN] cur position (X, Y) = (%.2f, %.2f), target (X, Y) = (%.2f, %.2f)",
                start_point(0), start_point(1), target_point(0), target_point(1));
    if (optimized_path.size() > 0){
        int path_start_id;
        int path_end_id;
        Eigen::Vector3d collision_pos;
        Eigen::Vector3d collision_start_point;
        Eigen::Vector3d collision_target_point;
        Eigen::Vector3d target_temp = optimized_path.back();
        RCLCPP_INFO(kLogger, "[Manager REPLAN] target_temp (X, Y) = (%.2f, %.2f)", target_temp(0), target_temp(1));

        bool collision = astar_path_finder->checkPathCollision(optimized_path, collision_pos, start_point,
                                                               collision_start_point, collision_target_point,
                                                               path_start_id, path_end_id);
        double target_distance = sqrt(pow(target_point.x() - target_temp.x(), 2) +
                               pow(target_point.y() - target_temp.y(), 2)) + 0.01;

        if(target_distance > 0.5){
            RCLCPP_WARN(kLogger, "[Manager REPLAN] target_distance: %.2f", target_distance);
        }
        if (collision) {
            std::vector<Eigen::Vector3d> local_path;
            RCLCPP_INFO(kLogger, "[Manager REPLAN] start local planning");
            local_path = localPathFinding(collision_start_point, collision_target_point);

            if ((local_path.size() == 0)) {
                RCLCPP_WARN(kLogger, "[Manager REPLAN] local plan fail, need global replan");
                if(!pathFinding(start_point, target_point, start_vel)){
                    return false;
                }else{
                    return true;
                }
            }
            else{
                local_path.insert(local_path.end(), optimized_path.begin() + path_end_id + 1, optimized_path.end());
                optimized_path = astar_path_finder->smoothTopoPath(local_path);
                local_optimize_path = local_path;

                RCLCPP_INFO(kLogger, "[Manager REPLAN] optimized path size is %zu", optimized_path.size());
                // 路径优化

                double reference_speed = isxtl? reference_desire_speedxtl : reference_desire_speed;

                path_smoother->init(optimized_path, start_vel, reference_speed);
                path_smoother->smoothPath();
                path_smoother->pathResample();
                final_path = path_smoother->getPath();
//                final_path = path_smoother->getSamplePath();
                if(final_path.size() < 2){
                    return false;
                }
                reference_path->setGlobalPath(start_vel, final_path,reference_a_max, reference_speed, isxtl);
                reference_path->getRefTrajectory(ref_trajectory, path_smoother->m_trapezoidal_time);
                if(ref_trajectory.size() < 2){
                    return false;
                }
                return true;
            }
        }
        else{
            if(!pathFinding(start_point, target_point, start_vel))
                return false;
            else
                return true;
        }

    }
    else {
        if(!pathFinding(start_point, target_point, start_vel))
            return false;
        else
            return true;
    }
}
