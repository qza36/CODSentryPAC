#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include "planner_config.hpp"
#include "trajectory_generation/GridNode.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include "trajectory_generation/GridMap.hpp"

/**
 * @brief A*路径搜索器
 *
 * 功能:
 * - 路径碰撞检测
 * - 拓扑路径平滑(剪枝优化)
 * - 可见性检查
 * - 地图可视化
 */
class AstarPathFinder
{
public:
    std::shared_ptr<GlobalMap> global_map;

    // ROS2发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_grid_map_vis_pub;

    AstarPathFinder() {};
    ~AstarPathFinder() {};

    /**
     * @brief 初始化A*搜索器
     * @param nh ROS2节点
     * @param _global_map 全局地图指针
     */
    void initGridMap(rclcpp::Node::SharedPtr nh, std::shared_ptr<GlobalMap> &_global_map);

    // ==================== 路径碰撞检测 ====================
    /**
     * @brief 检查路径是否与障碍物碰撞
     * @param optimized_path 待检查的路径
     * @param collision_pos [out] 碰撞点位置
     * @param cur_pos 当前位置
     * @param collision_start_point [out] 碰撞段起点
     * @param collision_target_point [out] 碰撞段终点
     * @param path_start_id [out] 碰撞段起点索引
     * @param path_end_id [out] 碰撞段终点索引
     * @return true=存在碰撞
     */
    bool checkPathCollision(const std::vector<Eigen::Vector3d> &optimized_path,
                            Eigen::Vector3d &collision_pos,
                            const Eigen::Vector3d &cur_pos,
                            Eigen::Vector3d &collision_start_point,
                            Eigen::Vector3d &collision_target_point,
                            int &path_start_id, int &path_end_id);

    /**
     * @brief 获取当前位置在路径中的索引
     */
    void getCurPositionIndex(const std::vector<Eigen::Vector3d> &optimized_path,
                             const Eigen::Vector3d &cur_pos,
                             int &cur_start_id);

    /**
     * @brief 检查单点周围是否有障碍物
     * @param path_point 检查点索引
     * @param check_swell 检查范围(栅格数)
     */
    bool checkPointCollision(Eigen::Vector3i path_point, int check_swell);

    // ==================== 路径优化 ====================
    /**
     * @brief 拓扑路径平滑(剪枝)
     *
     * 通过可见性检查对拓扑路径进行剪枝优化，
     * 去除不必要的中间点，生成更短的路径
     *
     * @param topo_path 原始拓扑路径
     * @return 平滑后的路径
     */
    std::vector<Eigen::Vector3d> smoothTopoPath(const std::vector<Eigen::Vector3d> &topo_path);

    /**
     * @brief 寻找附近的可行点
     * @param headPos 当前点
     * @param tailPos 目标点
     * @param best_point [out] 找到的可行点
     * @return true=找到可行点
     */
    bool getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point);

    /**
     * @brief 寻找附近最近的非障碍物点
     */
    bool findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num);

    // ==================== 可见性检查 ====================
    /**
     * @brief 两点间直线可见性检查
     *
     * 检查p1到p2的直线是否被障碍物遮挡，
     * 同时考虑高度变化约束(上坡≤0.12m, 下坡≤0.3m)
     *
     * @param p1 起点
     * @param p2 终点
     * @param colli_pt [out] 碰撞点
     * @param thresh 采样间隔
     * @return true=可见(无碰撞)
     */
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                   Eigen::Vector3d& colli_pt, double thresh);

    // ==================== 可视化 ====================
    void visGridMap();
    void visLocalGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool swell_flag);

    // ==================== 工具函数 ====================
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    // TODO: 实现找附近障碍物函数
    void findRadiusObstacle(double radius);
    void stayAwayObstacle(std::vector<Eigen::Vector3d> src_path, std::vector<Eigen::Vector3d> &optimized_path);

    Eigen::Vector3i goalIdx;
    rclcpp::Node::SharedPtr node_;
};

#endif
