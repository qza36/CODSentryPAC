#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "trajectory_generation/GridNode.hpp"
#include <memory>
#include "opencv2/opencv.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "planner_config.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
class GlobalMap
{
public:
    uint8_t *data; // data存放地图的障碍物信息（data[i]为1说明i这个栅格是障碍物栅格）
    uint8_t *l_data; // l_data存放局部地图的障碍物信息（l_data[i]为1说明i这个栅格是障碍物栅格）只用于判断地图对应高度的局部占用情况

    int GLX_SIZE, GLY_SIZE, GLZ_SIZE; // 地图尺寸
    int GLXYZ_SIZE, GLYZ_SIZE, GLXY_SIZE;
    size_t m_voxel_num; //GLXYZ_SIZE 搜索范围内的grid数量
    double m_resolution,m_inv_resolution; //分辨率 一个grid代表物理世界中多长的距离
    GridNodePtr **GridNodeMap;
    GridNodePtr **GridNodeLocalMap;  //local

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_local_cloud;
    Eigen::Vector3d odom_position; //位姿
    Eigen::Vector3d odom_posture; //什么用？

    bool m_local_update; //局部地图是否已经被规划
    std::vector<Eigen::Vector3d> topo_sample_map; //拓扑采样后的骨架？
    std::vector<Eigen::Vector3d> topo_keypoint; //

    std::vector<Eigen::Vector3d> second_heights_points; //处理桥洞高度
    void initGridMap(
        rclcpp::Node::SharedPtr node,
        const planner_manger::PlannerConfig& config
        );
    void occMap2Obs(const cv::Mat &occ_map);
    void topoSampleMap(cv::Mat &topo_map);
    cv::Mat swellOccMap(cv::Mat occ_map); //膨胀

    double getHeight(int idx_x, int idx_y);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    void coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx);
private:
    rclcpp::Node::SharedPtr m_node;
    double gl_xl, gl_yl, gl_zl;			   // 地图的左下角坐标
    double gl_xu, gl_yu, gl_zu;			   // 地图的右上角坐标


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
    void gazeboPoseCallback(const gazebo_msgs::msg::ModelStates::ConstSharedPtr &state);
    void gazeboCloudCallback(const sensor_msgs::msg::PointCloud2 &pointcloud2);
    void setObs(const double coord_x,const double coord_y);
};
#endif
