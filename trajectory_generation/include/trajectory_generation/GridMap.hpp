#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "trajectory_generation/GridNode.hpp"
#include <memory>
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "getparm_utils.hpp"
class GlobalMap
{
public:
    GlobalMap()
    {
        m_local_cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); //智能指针初始化

    }
    uint8_t *data;
    uint8_t *l_data;

    int GLX_SIZE,GLY_SIZE,GLZ_SIZE; //地图尺寸
    int GLXYZ_SIZE,GLYZ_SIZE,GLXY_SIZE;
    double m_resolution,m_inv_resolution; //分辨率

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
        Eigen::Vector3d global_xyz_l,
        Eigen::Vector3d global_xyz_u);
private:
    double gl_xl, gl_yl, gl_zl;	//lower		   // 地图的左下角坐标
    double gl_xu, gl_yu, gl_zu;	//upper		   // 地图的右上角坐标
};
#endif
