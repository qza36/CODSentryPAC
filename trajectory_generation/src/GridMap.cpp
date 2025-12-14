#include "trajectory_generation/GridMap.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/planner_config.hpp"
#include "trajectory_generation/MapLoader.hpp"

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
    data.resize(m_z_stride,0);
    l_data.resize(m_z_stride,0);
    /*设置2D搜索高度*/
    m_2d_search_height_high=config.search.search_height_max;
    m_2d_search_height_low=config.search.search_height_min;

    m_robot_radius=config.search.robot_radius;
    m_robot_radius_dash=config.search.robot_radius_dash;


    auto occ_map = MapLoader::loadOccMapData(config.map.occ_map_path);
    auto topo_map = MapLoader::loadTopoMapData(config.map.distance_map_path);

    m_gazebo_odometry_sub = m_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states",
        10,
        std::bind(&GlobalMap::gazeboPoseCallback,this,std::placeholders::_1));
    int swell_num = (int)(m_robot_radius/m_resolution)*2;
    int GL_SIZE = swell_num + (int)(m_search_radius/m_resolution);

    int x_size = m_grid_size(0);
    int y_size = m_grid_size(1);
    m_grid_node_pool.resize(x_size*y_size); //申请连续内存,初始化A*搜索地图
    for (int i =0; i< x_size;i++) {
        for (int j =0 ;j < y_size;j++) {
            int idx = i * y_size+j;
            Eigen::Vector3i tmpIdx(i,j,0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            m_grid_node_pool[idx] = GridNode(tmpIdx,pos);
        }
    }

    int x_size_local = m_grid_size(0);
    int y_size_local = m_grid_size(1);
    m_grid_node_pool_local.resize(x_size_local*y_size_local); //申请连续内存,初始化A*搜索地图
    for (int i =0; i< x_size_local;i++) {
        for (int j =0 ;j < y_size_local;j++) {
            int idx = i * y_size_local+j;
            Eigen::Vector3i tmpIdx_local(i,j,0);
            Eigen::Vector3d pos_local = gridIndex2coord(tmpIdx_local);
            m_grid_node_pool_local[idx] = GridNode(tmpIdx_local,pos_local);
        }
    }

}

Eigen::Vector3d GlobalMap::gridIndex2coord(const Eigen::Vector3i &index) {
    Eigen::Vector3d pt;
    pt(0) = ((double)index(0)+0.5)*m_resolution+m_gl_l(0);
    pt(1) = ((double)index(1)+0.5)*m_resolution+m_gl_l(1);
    pt(2) = ((double)index(2)+0.5)*m_resolution+m_gl_l(2);
    return pt;
}

Eigen::Vector3i GlobalMap::coord2girdIndex(const Eigen::Vector3d &pt) {
    Eigen::Vector3i idx;
    idx << std::min(std::max(int((pt(0) - m_gl_l(0)) * m_inv_resolution), 0), m_grid_size(0) - 1),
            std::min(std::max(int((pt(1) - m_gl_l(1)) * m_inv_resolution), 0), m_grid_size(1) - 1),
            std::min(std::max(int((pt(2) - m_gl_l(2)) * m_inv_resolution), 0), m_grid_size(2) - 1);

    return idx;
}

void GlobalMap::setObs(const double coord_x, const double coord_y) {
    if (coord_x <m_gl_l(0)||coord_y<m_gl_l(1)||
        coord_x >= m_gl_u(0)||coord_y >=m_gl_u(1)
    ) {
        return;
    }
    int idx_x = static_cast<int>((coord_x - m_gl_l(0))*m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - m_gl_l(1))*m_inv_resolution);

    data[idx_x*m_grid_size(1)+idx_y] = 1;

}

void GlobalMap::occMap2Obs(const map_utils::MapData &map) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt;
    for (int i =0; i<map.height;i++) {
        for (int j =0;j<map.width;j++) {
            int idx = i*map.width+j;
            uint8_t pixel_val = map.pixels[idx];
            if (pixel_val!=0) {
                double x = (j+0.5)*m_resolution;
                double y = (map.height-i-0.5)*m_resolution;
                this->setObs(x,y);
            }
        }
    }
}
