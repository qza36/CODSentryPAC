/**
 * @file replan_fsm.cpp
 * @brief 通用重规划状态机实现
 */

#include "trajectory_generation/replan_fsm.hpp"

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("trajectory_generation.fsm");
}

void ReplanFSM::init(rclcpp::Node::SharedPtr node)
{
    node_ = node;

    // 读取参数
    node_->declare_parameter("fsm.replan_thresh", 1.0);
    node_->declare_parameter("fsm.no_replan_thresh", 1.5);
    node_->declare_parameter("fsm.emergency_time", 2.0);
    node_->declare_parameter("fsm.exec_timer_period", 0.05);
    node_->declare_parameter("fsm.safety_timer_period", 0.1);

    replan_thresh_ = node_->get_parameter("fsm.replan_thresh").as_double();
    no_replan_thresh_ = node_->get_parameter("fsm.no_replan_thresh").as_double();
    emergency_time_ = node_->get_parameter("fsm.emergency_time").as_double();
    exec_timer_period_ = node_->get_parameter("fsm.exec_timer_period").as_double();
    safety_timer_period_ = node_->get_parameter("fsm.safety_timer_period").as_double();

    // 初始化规划管理器
    planner_manager::init(node_);

    // 初始化可视化
    visualization_ = std::make_unique<Visualization>();
    visualization_->init(node_);

    // 创建订阅者
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&ReplanFSM::odometryCallback, this, std::placeholders::_1));

    target_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&ReplanFSM::targetCallback, this, std::placeholders::_1));

    waypoint_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "waypoints", 10, std::bind(&ReplanFSM::waypointCallback, this, std::placeholders::_1));

    replan_flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "replan_flag", 10, std::bind(&ReplanFSM::replanFlagCallback, this, std::placeholders::_1));

    // 创建发布者
    traj_pub_ = node_->create_publisher<trajectory_generation::msg::TrajectoryPoly>("trajectory", 10);
    target_point_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("target_result", 10);
    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local();
    grid_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", qos_latched);

    // 创建定时器
    exec_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(exec_timer_period_),
        std::bind(&ReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(safety_timer_period_),
        std::bind(&ReplanFSM::safetyCheckCallback, this));

    RCLCPP_INFO(kLogger, "[FSM] Initialized, exec_period=%.2fs, safety_period=%.2fs",
                exec_timer_period_, safety_timer_period_);

    // 初始化完成后发布一次地图可视化
    visualization_->visGridMap(planner_manager::global_map);
}

// ==================== 状态机主循环 ====================
void ReplanFSM::execFSMCallback()
{
    switch (exec_state_) {
        case INIT: {
            if (have_odom_ && trigger_) {
                changeFSMState(WAIT_TARGET, "odom received");
            }
            break;
        }

        case WAIT_TARGET: {
            if (have_target_) {
                changeFSMState(GEN_NEW_TRAJ, "target received");
            }
            break;
        }

        case GEN_NEW_TRAJ: {
            start_pt_ = odom_pos_;
            start_vel_ = odom_vel_;

            // 检查起点是否有效
            Eigen::Vector3i start_idx = planner_manager::global_map->coord2gridIndex(start_pt_);
            if (planner_manager::global_map->isOccupied(start_idx, false)) {
                RCLCPP_WARN(kLogger, "[FSM] Start point (%.2f, %.2f) is in obstacle, waiting for valid odom...",
                            start_pt_.x(), start_pt_.y());
                break;
            }

            bool success = callPathPlanning();
            if (success) {
                changeFSMState(EXEC_TRAJ, "planning success");
                publishTrajectory();
                // 可视化
                visualization_->visFinalPath(planner_manager::optimized_path);
                visualization_->visReferencePath(planner_manager::ref_trajectory);
            } else {
                RCLCPP_WARN(kLogger, "[FSM] Planning failed, retry...");
                // 保持在GEN_NEW_TRAJ状态，下次循环重试
            }
            break;
        }

        case EXEC_TRAJ: {
            // 更新可视化
            visualization_->visCurPosition(odom_pos_);
            visualization_->visTargetPosition(end_pt_);
            publishTargetPoint();

            // 检查是否到达终点
            double dist_to_goal = (end_pt_ - odom_pos_).head<2>().norm();
            if (dist_to_goal < 0.3) {
                RCLCPP_INFO(kLogger, "[FSM] Goal reached!");
                have_target_ = false;
                changeFSMState(WAIT_TARGET, "goal reached");
                break;
            }

            // 检查是否需要重规划
            double dist_to_start = (start_pt_ - odom_pos_).head<2>().norm();
            if (dist_to_goal < no_replan_thresh_) {
                // 接近终点，不重规划
            } else if (dist_to_start < replan_thresh_) {
                // 刚开始执行，不重规划
            } else {
                // 触发周期性重规划
                changeFSMState(REPLAN_TRAJ, "periodic replan");
            }
            break;
        }

        case REPLAN_TRAJ: {
            start_pt_ = odom_pos_;
            start_vel_ = odom_vel_;

            // 检查起点是否有效
            Eigen::Vector3i start_idx = planner_manager::global_map->coord2gridIndex(start_pt_);
            if (planner_manager::global_map->isOccupied(start_idx, false)) {
                RCLCPP_WARN(kLogger, "[FSM] Replan start (%.2f, %.2f) is in obstacle, skip replan",
                            start_pt_.x(), start_pt_.y());
                changeFSMState(EXEC_TRAJ, "invalid start");
                break;
            }

            bool success = callReplan();
            if (success) {
                changeFSMState(EXEC_TRAJ, "replan success");
                publishTrajectory();
                if (!planner_manager::optimized_path.empty()) {
                    visualization_->visFinalPath(planner_manager::optimized_path);
                }
                if (!planner_manager::ref_trajectory.empty()) {
                    visualization_->visReferencePath(planner_manager::ref_trajectory);
                }
            } else {
                // 重规划失败，尝试全局规划
                RCLCPP_WARN(kLogger, "[FSM] Replan failed, try global planning");
                changeFSMState(GEN_NEW_TRAJ, "replan failed");
            }
            break;
        }

        case EMERGENCY_STOP: {
            // 等待速度降低
            if (odom_vel_.norm() < 0.1) {
                RCLCPP_INFO(kLogger, "[FSM] Emergency stop complete, replanning...");
                changeFSMState(GEN_NEW_TRAJ, "emergency cleared");
            }
            break;
        }
    }
}

// ==================== 安全检查 ====================
void ReplanFSM::safetyCheckCallback()
{
    if (exec_state_ != EXEC_TRAJ) return;

    bool collision = checkCollision();
    if (collision) {
        // 尝试快速重规划
        bool success = callReplan();
        if (success) {
            RCLCPP_WARN(kLogger, "[SAFETY] Collision detected, replanned");
            if (!planner_manager::optimized_path.empty()) {
                visualization_->visFinalPath(planner_manager::optimized_path);
            }
        } else {
            // 重规划失败，紧急停止
            RCLCPP_ERROR(kLogger, "[SAFETY] Emergency stop!");
            changeFSMState(EMERGENCY_STOP, "collision detected");
        }
    }
}

// ==================== 回调函数 ====================
void ReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_pos_.x() = msg->pose.pose.position.x;
    odom_pos_.y() = msg->pose.pose.position.y;
    odom_pos_.z() = msg->pose.pose.position.z;

    odom_vel_.x() = msg->twist.twist.linear.x;
    odom_vel_.y() = msg->twist.twist.linear.y;
    odom_vel_.z() = msg->twist.twist.linear.z;

    // 从四元数提取yaw
    auto& q = msg->pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    have_odom_ = true;
}

void ReplanFSM::targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    end_pt_.x() = msg->pose.position.x;
    end_pt_.y() = msg->pose.position.y;
    end_pt_.z() = 0.0;

    // 检查目标点是否在障碍物内
    Eigen::Vector3i idx = planner_manager::global_map->coord2gridIndex(end_pt_);
    if (planner_manager::global_map->isOccupied(idx, false)) {
        RCLCPP_ERROR(kLogger, "[FSM] Target (%.2f, %.2f) is inside obstacle! Ignoring.", end_pt_.x(), end_pt_.y());
        return;
    }

    RCLCPP_INFO(kLogger, "[FSM] New target: (%.2f, %.2f)", end_pt_.x(), end_pt_.y());

    have_target_ = true;

    // 如果正在执行，触发重规划
    if (exec_state_ == EXEC_TRAJ) {
        changeFSMState(REPLAN_TRAJ, "new target");
    }
}

void ReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) return;

    // 取最后一个点作为目标
    auto& pose = msg->poses.back().pose;
    end_pt_.x() = pose.position.x;
    end_pt_.y() = pose.position.y;
    end_pt_.z() = 0.0;

    RCLCPP_INFO(kLogger, "[FSM] Waypoint target: (%.2f, %.2f)", end_pt_.x(), end_pt_.y());

    have_target_ = true;

    if (exec_state_ == WAIT_TARGET) {
        changeFSMState(GEN_NEW_TRAJ, "waypoint received");
    } else if (exec_state_ == EXEC_TRAJ) {
        changeFSMState(REPLAN_TRAJ, "waypoint received");
    }
}

void ReplanFSM::replanFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && exec_state_ == EXEC_TRAJ) {
        RCLCPP_INFO(kLogger, "[FSM] External replan triggered");
        changeFSMState(REPLAN_TRAJ, "external trigger");
    }
}

// ==================== 状态机辅助函数 ====================
void ReplanFSM::changeFSMState(FSM_STATE new_state, const std::string& reason)
{
    last_state_ = exec_state_;
    exec_state_ = new_state;

    RCLCPP_INFO(kLogger, "[FSM] %s -> %s (%s)",
                stateToString(last_state_).c_str(),
                stateToString(new_state).c_str(),
                reason.c_str());
}

std::string ReplanFSM::stateToString(FSM_STATE state)
{
    switch (state) {
        case INIT: return "INIT";
        case WAIT_TARGET: return "WAIT_TARGET";
        case GEN_NEW_TRAJ: return "GEN_NEW_TRAJ";
        case EXEC_TRAJ: return "EXEC_TRAJ";
        case REPLAN_TRAJ: return "REPLAN_TRAJ";
        case EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

void ReplanFSM::printFSMState()
{
    RCLCPP_INFO(kLogger, "[FSM] State: %s, odom: %d, target: %d",
                stateToString(exec_state_).c_str(), have_odom_, have_target_);
}

// ==================== 规划函数 ====================
bool ReplanFSM::callPathPlanning()
{
    return planner_manager::pathFinding(start_pt_, end_pt_, start_vel_);
}

bool ReplanFSM::callReplan()
{
    return planner_manager::replanFinding(start_pt_, end_pt_, start_vel_);
}

bool ReplanFSM::checkCollision()
{
    if (planner_manager::optimized_path.empty()) return false;

    Eigen::Vector3d collision_pos, collision_start, collision_end;
    int start_id, end_id;

    return planner_manager::astar_path_finder->checkPathCollision(
        planner_manager::optimized_path,
        collision_pos, odom_pos_,
        collision_start, collision_end,
        start_id, end_id);
}

void ReplanFSM::publishTrajectory()
{
    auto& ref_path = planner_manager::reference_path;
    if (!ref_path || ref_path->m_trapezoidal_time.empty()) {
        RCLCPP_WARN(kLogger, "[FSM] No trajectory to publish");
        return;
    }

    trajectory_generation::msg::TrajectoryPoly msg;
    msg.start_time = node_->now();
    msg.motion_mode = 0;

    int num_segments = ref_path->m_trapezoidal_time.size();
    for (int i = 0; i < num_segments; ++i) {
        msg.duration.push_back(ref_path->m_trapezoidal_time[i]);

        // 每段4个系数: c0 + c1*t + c2*t^2 + c3*t^3
        for (int j = 0; j < 4; ++j) {
            msg.coef_x.push_back(ref_path->m_polyMatrix_x(i, j));
            msg.coef_y.push_back(ref_path->m_polyMatrix_y(i, j));
        }
    }

    traj_pub_->publish(msg);
    RCLCPP_INFO(kLogger, "[FSM] Published trajectory with %d segments", num_segments);
}

void ReplanFSM::publishTargetPoint()
{
    geometry_msgs::msg::Point msg;
    msg.x = end_pt_.x();
    msg.y = end_pt_.y();
    msg.z = end_pt_.z();
    target_point_pub_->publish(msg);
}

void ReplanFSM::publishGridMap()
{
    // TODO: 从global_map获取点云数据并发布
    // 当前为占位实现，需要GridMap提供点云转换接口
}
