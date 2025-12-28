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

/**
 * @brief 全局栅格地图类
 *
 * 功能:
 * - 管理全局和局部栅格地图
 * - 处理障碍物检测和膨胀
 * - 支持桥洞等复杂地形的二层高度处理
 * - 提供坐标转换接口
 */
class GlobalMap
{
public:
    // ==================== 地图数据 ====================
    uint8_t *data;    // 全局障碍物数据 (1=障碍物, 0=自由)
    uint8_t *l_data;  // 局部障碍物数据，用于动态障碍物检测

    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;  // 地图栅格尺寸 (x, y, z方向)
    int GLXYZ_SIZE, GLYZ_SIZE, GLXY_SIZE;  // 预计算的尺寸乘积
    size_t m_voxel_num;     // 总栅格数量
    double m_resolution;    // 栅格分辨率 (m/grid)
    double m_inv_resolution;// 分辨率倒数，用于加速计算

    GridNodePtr **GridNodeMap;       // 全局栅格节点二维数组
    GridNodePtr **GridNodeLocalMap;  // 局部栅格节点二维数组

    // ==================== 状态数据 ====================
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_local_cloud;  // 局部点云
    Eigen::Vector3d odom_position;  // 里程计位置
    Eigen::Vector3d odom_posture;   // 里程计姿态

    bool m_local_update;  // 局部地图更新标志

    // ==================== 拓扑采样数据 ====================
    std::vector<Eigen::Vector3d> topo_sample_map;  // 拓扑采样点(骨架点)
    std::vector<Eigen::Vector3d> topo_keypoint;    // 拓扑关键点

    // ==================== 桥洞处理 ====================
    std::vector<Eigen::Vector3d> second_heights_points;  // 二层高度点集合

    // ==================== 公共接口 ====================
    /**
     * @brief 初始化栅格地图
     * @param node ROS2节点指针
     * @param config 规划器配置
     */
    void initGridMap(rclcpp::Node::SharedPtr node, const planner_manager::PlannerConfig& config);

    /**
     * @brief 将占用地图转换为障碍物
     * @param occ_map OpenCV格式的占用地图
     */
    void occMap2Obs(const cv::Mat &occ_map);

    /**
     * @brief 从拓扑地图提取采样点
     * @param topo_map 拓扑地图
     */
    void topoSampleMap(cv::Mat &topo_map);

    /**
     * @brief 膨胀占用地图
     * @param occ_map 原始占用地图
     * @return 膨胀后的地图
     */
    cv::Mat swellOccMap(cv::Mat occ_map);

    /**
     * @brief 处理桥洞区域的二层高度
     */
    void processSecondHeights();

    /**
     * @brief 重置已使用的栅格状态
     */
    void resetUsedGrids();

    // ==================== 坐标转换 ====================
    double getHeight(int idx_x, int idx_y);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    void coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx);

    // ==================== 障碍物处理 ====================
    /**
     * @brief 将局部点云转换为障碍物
     * @param cloud 点云数据
     * @param swell_flag 是否膨胀
     * @param current_position 当前位置
     */
    void localPointCloudToObstacle(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                   const bool &swell_flag,
                                   const Eigen::Vector3d current_position);

    // ==================== 占用查询 ====================
    bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z, bool second_height) const;
    bool isOccupied(const Eigen::Vector3i &index, bool second_height) const;
    bool isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isLocalOccupied(const Eigen::Vector3i &index) const;
    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isFree(const Eigen::Vector3i &index) const;

    /**
     * @brief 设置动态障碍物膨胀半径
     */
    void setRadiusDash(const double dash_radius);

private:
    rclcpp::Node::SharedPtr m_node;
    Eigen::Vector3i current_position_index;

    // 地图边界
    double gl_xl, gl_yl, gl_zl;  // 左下角坐标
    double gl_xu, gl_yu, gl_zu;  // 右上角坐标

    // 高度相关参数
    double m_height_bias;                   // 高度偏移
    double m_height_interval;               // 高度区间
    double m_height_threshold;              // 高度阈值
    double m_height_sencond_high_threshold; // 二层高度阈值
    double m_2d_search_height_low;          // 2D搜索高度下限
    double m_2d_search_height_high;         // 2D搜索高度上限

    // 机器人参数
    double m_robot_radius;       // 碰撞检测半径
    double m_robot_radius_dash;  // 动态障碍物检测半径
    double m_search_radius;      // 搜索半径

    // ROS2订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_local_pointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr gazebo_point_sub;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr m_gazebo_odometry_sub;

    // ROS2发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub;

    // 私有方法
    void gazeboPoseCallback(const gazebo_msgs::msg::ModelStates::ConstSharedPtr &state);
    void gazeboCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud2);
    void setObs(const double coord_x, const double coord_y);
    void resetGrid(GridNodePtr ptr);
    void localSetObs(const double coord_x, const double coord_y, const double coord_z);
};

#endif
