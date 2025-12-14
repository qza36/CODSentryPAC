#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "trajectory_generation/GridNode.hpp"
#include <memory>

#include "MapData.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "planner_config.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
class GlobalMap
{
public:
    GlobalMap()
    {
        m_local_cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); //智能指针初始化
    }
    std::vector<uint8_t> data; // data存放地图的障碍物信息（data[i]为1说明i这个栅格是障碍物栅格）
    std::vector<uint8_t> l_data; // l_data存放局部地图的障碍物信息（l_data[i]为1说明i这个栅格是障碍物栅格）只用于判断地图对应高度的局部占用情况
    std::vector<GridNode> m_grid_node_pool; //节点池
    std::vector<GridNode> m_grid_node_pool_local; //节点池

    Eigen::Vector3i m_grid_size; //GLX_SIZE GLY_SIZE GLZ_SIZE,A*拓展节点范围
    int m_z_stride; //GLXY_SIZE z轴上的步长
    int m_y_stride; //GLX_SZIE y轴上的步长
    size_t m_voxel_num; //GLXYZ_SIZE 搜索范围内的grid数量
    double m_resolution,m_inv_resolution; //分辨率 一个grid代表物理世界中多长的距离
    GridNodePtr **GridNodeMap;
    GridNodePtr **GridNodeLocalMap;  //local

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_local_cloud;
    Eigen::Vector3d odom_position; //位姿
    Eigen::Vector3d odom_poseture; //什么用？

    bool m_local_update; //局部地图是否已经被规划
    std::vector<Eigen::Vector3d> topo_sample_map; //拓扑采样后的骨架？
    std::vector<Eigen::Vector3d> topo_keypoint; //

    std::vector<Eigen::Vector3d> second_heights_points; //处理桥洞高度
    void initGridMap(
        rclcpp::Node::SharedPtr node,
        const planner_manger::PlannerConfig& config
        );
    void occMap2Obs(const map_utils::MapData &map);
    void topoSampleMap(const map_utils::MapData &map);

    double getHeight(int idx_x, int idx_y);
private:
    rclcpp::Node::SharedPtr m_node;
    Eigen::Vector3d m_gl_l;
    Eigen::Vector3d m_gl_u;


    double m_height_bias;
    double m_height_interval;
    double m_height_threshold;
    double m_height_sencond_high_threshold;
    double m_2d_search_height_low;	// 2维规划障碍物搜索高度最低点
    double m_2d_search_height_high; // 2维规划障碍物搜索高度最高点

    double m_robot_radius;			// 机器人的碰撞检测直径 建议<0.38
    double m_robot_radius_dash;		// 机器人的动态障碍物冲卡检测直径 建议<0.25
    double m_search_radius;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_local_pointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr gazebo_point_sub;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr m_gazebo_odometry_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub;
    void gazeboPoseCallback(const gazebo_msgs::msg::ModelStates &state);
    void gazeboCloudCallback(const sensor_msgs::msg::PointCloud2 &pointcloud2);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    Eigen::Vector3i coord2girdIndex(const Eigen::Vector3d &coord);
    void coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx);
    void setObs(const double coord_x,const double coord_y);
};
#endif
