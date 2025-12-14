#include "trajectory_generation/GridMap.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/planner_config.hpp"
#include "opencv4/opencv2/opencv.hpp"
void GlobalMap::initGridMap(
    rclcpp::Node::SharedPtr node,
    const planner_manger::PlannerConfig& config
)
{
    m_node = node;
    m_height_bias = config.map.height_bias;

    //获取地图边界
    m_gl_l = config.map.map_lower_point;
    m_gl_u = config.map.map_upper_point;
    //获得搜索范围
    m_grid_size = config.map.map_grid_size;
    //搜索步长
    m_y_stride = m_grid_size(0);
    m_z_stride = m_grid_size(0) * m_grid_size(1);
    m_voxel_num = (size_t)m_grid_size(0) * (size_t)m_grid_size(1) * (size_t)m_grid_size(2);

    m_resolution = 1.0 /config.map.map_resolution;
    //初始化地图
    data->resize(m_z_stride,0);
    l_data->resize(m_z_stride,0);
    /*设置2D搜索高度*/
    m_2d_search_height_high=config.search.search_height_max;
    m_2d_search_height_low=config.search.search_height_min;

    m_robot_radius=config.search.robot_radius;
    m_robot_radius_dash=config.search.robot_radius_dash;


    cv::Mat occ_map;
    cv::Mat topo_map;
    occ_map = cv::imread(config.map.occ_map_path,cv::IMREAD_GRAYSCALE); //直接以灰度图像读取
    topo_map= cv::imread(config.map.distance_map_path,cv::IMREAD_GRAYSCALE); //直接以灰度图像读取


}
