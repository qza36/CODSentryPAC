#include "trajectory_generation/GridMap.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/planner_config.hpp"

void GlobalMap::initGridMap(
    rclcpp::Node::SharedPtr node,
    const planner_manger::PlannerConfig& config
)
{
    m_node = node;
    m_height_bias = config.map.height_bias;

    //获取地图边界
    config.map.map_upper_point;
    gl_xl = config.map.map_lower_point(0);
    gl_yl = config.map.map_lower_point(1);
    gl_zl = config.map.map_lower_point(2);
    gl_xu = config.map.map_upper_point(0);
    gl_yu = config.map.map_upper_point(1) ;
    gl_zu = config.map.map_upper_point(2);

    GLX_SIZE = config.map.map_grid_size(0);
    GLY_SIZE = config.map.map_grid_size(1);
    GLZ_SIZE = config.map.map_grid_size(2);
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    m_resolution = 1.0 /config.map.map_resolution;
    //初始化地图
    data = new uint8_t[GLXY_SIZE];
    l_data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
    memset(l_data, 0, GLXY_SIZE * sizeof(uint8_t));
    /*设置2D搜索高度*/
    m_2d_search_height_high=config.search.search_height_max;
    m_2d_search_height_low=config.search.search_height_min;

    m_robot_radius=config.search.robot_radius;
    m_robot_radius_dash=config.search.robot_radius_dash;

    std::string occ_map_file = config.map.occ_map_path;
    std::string topo_map_file = config.map.topo_map_path;
    cv::Mat occ_map = cv::imread(occ_map_file,cv::IMREAD_GRAYSCALE);
    cv::Mat topo_map = cv::imread(topo_map_file,cv::IMREAD_GRAYSCALE);


    m_gazebo_odometry_sub = m_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states",
        10,
        std::bind(&GlobalMap::gazeboPoseCallback,this,std::placeholders::_1));
    int swell_num = (int)(m_robot_radius/m_resolution)*2;
    int GL_SIZE = swell_num + (int)(m_search_radius/m_resolution);

    GridNodeMap = new GridNodePtr *[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++) {
        GridNodeMap[i] = new GridNodePtr [GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++){
            Eigen::Vector3i tmpIdx(i, j, 0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }

    GridNodeLocalMap = new GridNodePtr *[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeLocalMap[i] = new GridNodePtr[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            Eigen::Vector3i tmpIdx(i, j, 0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeLocalMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }
    occ_map = swellOccMap(occ_map);
    occMap2Obs(occ_map);
    topoSampleMap(topo_map);
    m_local_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

Eigen::Vector3d GlobalMap::gridIndex2coord(const Eigen::Vector3i &index) {
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * m_resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * m_resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * m_resolution + gl_zl;

    return pt;
}

Eigen::Vector3i GlobalMap::coord2gridIndex(const Eigen::Vector3d &pt) {
    Eigen::Vector3i idx;
    idx << std::min(std::max(int((pt(0) - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1),
            std::min(std::max(int((pt(1) - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1),
            std::min(std::max(int((pt(2) - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

void GlobalMap::setObs(const double coord_x, const double coord_y) {
    if (coord_x < gl_xl || coord_y < gl_yl  ||
        coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);

    /* 将障碍物映射为容器里值为1的元素 */
    data[idx_x * GLY_SIZE + idx_y] = 1;

}

void GlobalMap::occMap2Obs(const cv::Mat &occ_map) {
    pcl::PointXYZ pt;
    for (int i =0; i<occ_map.rows;i++) {
        for (int j =0;j<occ_map.cols;j++) {
            if (occ_map.at<uchar>(i,j)!=0) {
                pt.x = (j+0.5) * m_resolution;
                pt.y = (occ_map.rows - i - 0.5) * m_resolution;
                setObs(pt.x, pt.y);
            }
        }
    }
}
void GlobalMap::topoSampleMap(cv::Mat &topo_map) {
    RCLCPP_INFO(m_node->get_logger(),"start topo sample map");
    Eigen::Vector3d topoMapTemp;
    for(int i = 0; i < topo_map.rows; i++){
        for(int j = 0; j < topo_map.cols; j++){
            if(topo_map.at<uchar>(i, j) > 10){
                topoMapTemp.x() = (j + 0.5) * m_resolution;
                topoMapTemp.y() = (topo_map.rows - i - 0.5) * m_resolution;
                topoMapTemp.z() = 0.0;

                int keypoint = 0;
                for(int idx = -1; idx < 2; idx++){
                    for(int idy = -1; idy < 2; idy++){
                        if(topo_map.at<uchar>(i + idx, j + idy) > 100){
                            keypoint ++ ;
                        }
                    }
                }
                if(keypoint > 3){  // 多路径交汇点作为topo的关键点必进入topo search中
                    int pt_idx, pt_idy, pt_idz;
                    coord2gridIndex(topoMapTemp.x(), topoMapTemp.y(), topoMapTemp.z(), pt_idx, pt_idy, pt_idz);
                    topoMapTemp.z() = getHeight(pt_idx, pt_idy);
                    topo_keypoint.push_back(topoMapTemp);

                    if(GridNodeMap[pt_idx][pt_idy]->exist_second_height){  // 桥洞区域的所有点双倍采样，分别设置桥上和桥下点
                        topoMapTemp.z() = GridNodeMap[pt_idx][pt_idy]->second_height;
                        topo_keypoint.push_back(topoMapTemp);
                    }

                    for(int idx = -1; idx < 2; idx++){
                        for(int idy = -1; idy < 2; idy++){
                            topo_map.at<uchar>(i + idx, j + idy) = 0;
                        }
                    }
                }
                if(keypoint == 3){
                    int pt_idx, pt_idy, pt_idz;
                    coord2gridIndex(topoMapTemp.x(), topoMapTemp.y(), topoMapTemp.z(), pt_idx, pt_idy, pt_idz);
                    topoMapTemp.z() = getHeight(pt_idx, pt_idy);
                    topo_sample_map.push_back(topoMapTemp);

                    if(GridNodeMap[pt_idx][pt_idy]->exist_second_height){  //桥洞同等处理
                        topoMapTemp.z() = GridNodeMap[pt_idx][pt_idy]->second_height;
                        topo_sample_map.push_back(topoMapTemp);
                    }

                }
            }
        }
    }
    RCLCPP_WARN(m_node->get_logger(),"[GridMap] topo_sample_map size: %lu", topo_sample_map.size());
    RCLCPP_WARN(m_node->get_logger(),"[GridMap] topo_keypoint size: %d", topo_keypoint.size());
}
void GlobalMap::coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx) {
    x_idx = std::min(std::max(int((pt_x - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1);
    y_idx = std::min(std::max(int((pt_y - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1);
    z_idx = std::min(std::max(int((pt_z - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);
}

double GlobalMap::getHeight(int idx_x, int idx_y) {
    if (idx_x > GLX_SIZE || idx_y > GLY_SIZE)
        return 0.0;
    return GridNodeMap[idx_x][idx_y]->height;
}
