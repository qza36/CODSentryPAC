/**
 * @file replan_fsm.hpp
 * @brief 通用重规划状态机 - 适用于低速移动机器人
 *
 * 参考: HKU EGO-Planner FSM设计
 * 简化: 移除无人机/哨兵专用功能，适配地面机器人
 */

#ifndef REPLAN_FSM_HPP
#define REPLAN_FSM_HPP

#include <Eigen/Eigen>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_generation/plannerManager.hpp"
#include "trajectory_generation/visualization_utils.hpp"

/**
 * @brief 重规划状态机
 *
 * 状态转换:
 * INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ ⇄ REPLAN_TRAJ
 *                                         ↓
 *                                   EMERGENCY_STOP
 */
class ReplanFSM
{
public:
    ReplanFSM() = default;
    ~ReplanFSM() = default;

    /**
     * @brief 初始化状态机
     * @param node ROS2节点指针
     */
    void init(rclcpp::Node::SharedPtr node);

    // ==================== 状态定义 ====================
    enum FSM_STATE {
        INIT,           // 初始化，等待里程计
        WAIT_TARGET,    // 等待目标点
        GEN_NEW_TRAJ,   // 生成新轨迹
        EXEC_TRAJ,      // 执行轨迹
        REPLAN_TRAJ,    // 重规划
        EMERGENCY_STOP  // 紧急停止
    };

private:
    rclcpp::Node::SharedPtr node_;

    // ==================== 状态变量 ====================
    FSM_STATE exec_state_ = INIT;
    FSM_STATE last_state_ = INIT;

    // ==================== 标志位 ====================
    bool have_odom_ = false;
    bool have_target_ = false;
    bool trigger_ = true;  // 允许规划触发

    // ==================== 机器人状态 ====================
    Eigen::Vector3d odom_pos_;   // 当前位置
    Eigen::Vector3d odom_vel_;   // 当前速度
    double odom_yaw_ = 0.0;      // 当前航向

    Eigen::Vector3d start_pt_;   // 规划起点
    Eigen::Vector3d start_vel_;  // 起点速度
    Eigen::Vector3d end_pt_;     // 目标点

    // ==================== 参数 ====================
    double replan_thresh_ = 1.0;      // 重规划距离阈值(m)
    double no_replan_thresh_ = 1.5;   // 接近终点不重规划阈值(m)
    double emergency_time_ = 2.0;     // 紧急停止时间窗口(s)
    double exec_timer_period_ = 0.05; // 主循环周期(s) 20Hz
    double safety_timer_period_ = 0.1;// 安全检查周期(s) 10Hz

    // ==================== 定时器 ====================
    rclcpp::TimerBase::SharedPtr exec_timer_;    // 主状态机定时器
    rclcpp::TimerBase::SharedPtr safety_timer_;  // 安全检查定时器

    // ==================== 订阅者 ====================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;

    // ==================== 可视化 ====================
    std::unique_ptr<Visualization> visualization_;

    // ==================== 回调函数 ====================
    void execFSMCallback();
    void safetyCheckCallback();
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // ==================== 状态机辅助函数 ====================
    void changeFSMState(FSM_STATE new_state, const std::string& reason);
    std::string stateToString(FSM_STATE state);
    void printFSMState();

    // ==================== 规划函数 ====================
    bool callPathPlanning();
    bool callReplan();
    bool checkCollision();
};

#endif
