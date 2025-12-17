#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H
#include <iostream>

#include "planner_config.hpp"
#include "trajectory_generation/GridNode.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include "trajectory_generation/GridMap.hpp"
class AstarPathFinder
{
public:
    std::shared_ptr<GlobalMap> global_map;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_grid_map_vis_pub;

    AstarPathFinder(){};
    ~AstarPathFinder(){};
    void findRadiusObstacle(double radius); // TODO 找附近的障碍物函数实现

    void visGridMap();  // 全局地图可视化
    void visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag);  //局部地图

    void stayAwayObstacle(std::vector<Eigen::Vector3d> src_path, std::vector<Eigen::Vector3d> &optimized_path);
    void initGridMap(rclcpp::Node::SharedPtr nh, std::shared_ptr<GlobalMap> &_global_map);

    /**
     * @brief 检查优化完成后路径是否被障碍物遮挡
     * @param optimized_path
     * @param collision_pos
     * @param cur_pos
     * @param collision_start_point
     * @param collision_target_point
     * @param path_start_id
     * @param path_end_id
     * @return
     */
    bool checkPathCollision(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d &collision_pos, Eigen::Vector3d cur_pos,
                            Eigen::Vector3d &collision_start_point, Eigen::Vector3d &collision_target_point,
                            int &path_start_id, int &path_end_id);
    /**
     *@brief 获得当前位置在路径中的位置
     */
    void getCurPositionIndex(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d cur_pos, int &cur_start_id);

    bool checkPointCollision(Eigen::Vector3i path_point, int check_swell);							  /// 检查路径是否可行
    bool findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num);	  /// 找附近最近的非障碍物点
    bool getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point);

    std::vector<Eigen::Vector3d> smoothTopoPath(std::vector<Eigen::Vector3d> topo_path);

    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& colli_pt, double thresh);
    Eigen::Vector3i goalIdx;
    rclcpp::Node::SharedPtr node_;
};
#endif
