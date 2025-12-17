#include "trajectory_generation/GridMap.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/planner_config.hpp"
#include <pcl/common/transforms.h>

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
    std::string distance_map_file = config.map.distance_map_path;
    cv::Mat occ_map = cv::imread(occ_map_file,cv::IMREAD_GRAYSCALE);
    cv::Mat topo_map = cv::imread(distance_map_file,cv::IMREAD_GRAYSCALE);


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
    processSecondHeights();
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

void GlobalMap::gazeboPoseCallback(const gazebo_msgs::msg::ModelStates::ConstSharedPtr &state) {
    int robot_namespace_id = 0;
    for(int i = 0;i < state->name.size();i++){
        if(state->name[i] == "mbot"){
            robot_namespace_id = i;
            break;
        }
    }
    odom_position(0) = state->pose[robot_namespace_id].position.x;
    odom_position(1) = state->pose[robot_namespace_id].position.y;
    odom_position(2) = state->pose[robot_namespace_id].position.z;

    /*获得四元数*/
    double x = state->pose[robot_namespace_id].orientation.x;
    double y = state->pose[robot_namespace_id].orientation.y;
    double z = state->pose[robot_namespace_id].orientation.z;
    double w = state->pose[robot_namespace_id].orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);

    odom_posture(2) = atan2f(siny_cosp, cosy_cosp);
    odom_posture(1) = asin(2 * (w * y - x * z));
    odom_posture(0) = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

cv::Mat GlobalMap::swellOccMap(cv::Mat occ_map) {
    cv::Mat occ = cv::Mat::zeros(occ_map.rows, occ_map.cols, CV_8UC1);
    int swell_num = (int) (m_robot_radius / m_resolution);
    for (int i = 0; i < occ_map.rows; i++) {
        for (int j = 0; j < occ_map.cols; j++) {
            if (occ_map.at<uchar>(i, j) > 10)
            {   // 膨胀直接画圆
                cv::circle(occ, cv::Point(j, i), swell_num, cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    RCLCPP_INFO(m_node->get_logger(),"map swell done");
    return occ;
}

void GlobalMap::gazeboCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point) {
       /***
     * gazebo 雷达数据回调
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);
    local_cloud_3d->clear();
    m_local_cloud->clear();

    pcl::fromROSMsg(*point, *local_cloud_3d);

    Eigen::Matrix3d R_x; // 计算旋转矩阵的X分量
    R_x << 1, 0, 0,
            0, cos(odom_posture[0]), -sin(odom_posture[0]),
            0, sin(odom_posture[0]), cos(odom_posture[0]);

    Eigen::Matrix3d R_y; // 计算旋转矩阵的Y分量
    R_y << cos(odom_posture[1]), 0, sin(odom_posture[1]),
            0, 1, 0,
            -sin(odom_posture[1]), 0, cos(odom_posture[1]);

    Eigen::Matrix3d R_z; // 计算旋转矩阵的Z分量
    R_z << cos(odom_posture[2]), -sin(odom_posture[2]), 0,
            sin(odom_posture[2]), cos(odom_posture[2]), 0,
            0, 0, 1;
    Eigen::Matrix3d R = R_z * R_y * R_x;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = odom_position;

    // 定义输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 进行坐标系转换
    pcl::transformPointCloud(*local_cloud_3d, *output_cloud, transform);

    clock_t time_1 = clock();

    /* 点云区域筛选 */
    pcl::PointXYZ pt;
    pcl::PointXYZ pt_3d;
    int point_size = 0;

    for (int i = 0; i < (int) output_cloud->points.size(); i++)
    {
        pt_3d = output_cloud->points[i];
        /// 在范围内的点云
        if (pt_3d.z > m_2d_search_height_low && pt_3d.z < m_2d_search_height_high && (abs(pt_3d.x - odom_position(0)) < m_search_radius) && (abs(pt_3d.y - odom_position(1)) < m_search_radius))
        {
            pt.x = pt_3d.x;
            pt.y = pt_3d.y;
            pt.z = pt_3d.z + 0.2;
            m_local_cloud->points.push_back(pt);
            point_size++;
        }
    }
    clock_t time_2 = clock();
    localPointCloudToObstacle(*m_local_cloud, true, odom_position);

}

void GlobalMap::localPointCloudToObstacle(const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool &swell_flag, const Eigen::Vector3d current_position) {
    current_position_index = coord2gridIndex(current_position);  // 当前位置的栅格坐标
    pcl::PointXYZ pt;
    memset(l_data, 0, GLXY_SIZE * sizeof(uint8_t));  // TODO 这里直接reset，因此桥洞标志位需要不同的reset方式
    resetUsedGrids();

    if (!swell_flag){

        for (int idx = 0; idx < (int)cloud.points.size(); idx++){
            pt = cloud.points[idx];
            localSetObs(pt.x, pt.y, pt.z);
        }
    }
    else{
        /* 膨胀处理 */

        for (int idx = 0; idx < (int)cloud.points.size(); idx++){
            pt = cloud.points[idx];
            int swell_num = (int)(m_robot_radius_dash / m_resolution);
            if (pt.x < gl_xl || pt.y < gl_yl || pt.z < gl_zl ||
                pt.x >= gl_xu || pt.y >= gl_yu || pt.z >= gl_zu)
                continue;
            int idx_x = static_cast<int>((pt.x - gl_xl) * m_inv_resolution);
            int idx_y = static_cast<int>((pt.y - gl_yl) * m_inv_resolution);
            int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);
            bool in_bridge_cave = false;

            if(!GridNodeMap[idx_x][idx_y]->exist_second_height||!GridNodeMap[idx_x][idx_y]->second_local_swell){  // 非桥洞区域正常判断
                if (GridNodeMap[idx_x][idx_y]->height + m_height_threshold > pt.z || GridNodeMap[idx_x][idx_y]->height + 1.0 < pt.z)
                    continue;

            } else if(GridNodeMap[idx_x][idx_y]->second_local_swell){  // 如果在桥洞里一样的处理，只不过地图预设高度为second_height
                in_bridge_cave = true;
                if (GridNodeMap[idx_x][idx_y]->second_height + m_height_threshold > pt.z || GridNodeMap[idx_x][idx_y]->second_height + m_height_sencond_high_threshold < pt.z)
                    continue;
            }
            if(isOccupied(idx_x, idx_y, idx_z, false)){  // TODO 这里暂时处理为已经全局地图下被占用的情况下不再重新膨胀
                continue;
            }

            /// TODO 膨胀这里有点问题2024.6.18
            /// 通过高度地图阈值筛选之后膨胀设计全局地图的占用栅格数据，注意我们不需要设置局部地图高度
            for (int i = -swell_num; i <= swell_num; i++){
                for (int j = -swell_num; j <= swell_num; j++){
                    double temp_x = i * m_resolution + pt.x;
                    double temp_y = j * m_resolution + pt.y;
                    double temp_z = pt.z;

                    int idx = static_cast<int>((temp_x - gl_xl) * m_inv_resolution);
                    int idy = static_cast<int>((temp_y - gl_yl) * m_inv_resolution);
                    idx = std::max(idx, 0);
                    idy = std::max(idy, 0);
                    // 膨胀后的点要判断是不是在桥洞区域，以及如果是非桥洞点膨胀得到的点都不应该视为占用
                    if(GridNodeMap[idx_x][idx_y]->exist_second_height && !GridNodeMap[idx][idy]->exist_second_height){
                        continue;
                    }

                    if(!GridNodeMap[idx_x][idx_y]->exist_second_height && GridNodeMap[idx][idy]->exist_second_height){
                        continue;
                    }
                    localSetObs(temp_x, temp_y, temp_z);
                    /* 利用点云坐标为栅格地图设置障碍物 */

                }
            }
        }
    }
}
void GlobalMap::processSecondHeights()
{   // TODO 处理桥洞的高度,非常trick的方式，可以考虑手动指定桥洞位置的凡是
    while(!second_heights_points.empty()){
        Eigen::Vector3d temp_point = second_heights_points.back();
        double x_max = temp_point.x();
        double x_min = temp_point.x();
        double y_max = temp_point.y();
        double y_min = temp_point.y();
        second_heights_points.pop_back();
        for(std::vector<Eigen::Vector3d>::iterator it = second_heights_points.begin(); it != second_heights_points.end(); it++){
            if(std::sqrt((temp_point.x() - (*it).x()) * (temp_point.x() - (*it).x())
                        + (temp_point.y() - (*it).y()) * (temp_point.y() - (*it).y())) < 4.0){
                x_max = std::max(x_max, (*it).x());
                x_min = std::min(x_min, (*it).x());
                y_max = std::max(y_max, (*it).y());
                y_min = std::min(y_min, (*it).y());
                it = second_heights_points.erase(it);
                it--;
                        }
        }
        std::cout<<"xmax: "<<x_max<<" xmin: "<<x_min<<" ymax: "<<y_max<<" ymin: "<<y_min<<std::endl;

        int idx_x_max = static_cast<int>((x_max - gl_xl + 0.01) * m_inv_resolution);
        int idx_y_max = static_cast<int>((y_max - gl_yl + 0.01) * m_inv_resolution);
        int idx_x_min = static_cast<int>((x_min - gl_xl + 0.01) * m_inv_resolution);
        int idx_y_min = static_cast<int>((y_min - gl_yl + 0.01) * m_inv_resolution);
        for(int i = idx_x_min; i<=idx_x_max; i++) {
            for (int j = idx_y_min; j <= idx_y_max; j++) {
                GridNodeMap[i][j]->second_local_swell=true;
                for(int k = -2; k <= 2; k++){
                    for(int m = -2; m <= 2; m++){
                        GridNodeMap[i + k][j + m]->exist_second_height = true;
                    }
                }

            }
        }
    }
}
void GlobalMap::resetUsedGrids() {
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
        {
            resetGrid(GridNodeMap[i][j]);
        }
}
inline void GlobalMap::resetGrid(GridNodePtr ptr) {
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
    ptr->second_local_occupancy = false;
}
void GlobalMap::localSetObs(const double coord_x, const double coord_y, const double coord_z) {
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
    coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);
    int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);

    if(GridNodeMap[idx_x][idx_y]->exist_second_height){
        // 在桥洞位置，我们需要根据局部点云的高度和地图的高度差来判断是否桥洞被占用了还是高地被占用了
        if(coord_z > GridNodeMap[idx_x][idx_y]->second_height + m_height_threshold && coord_z < GridNodeMap[idx_x][idx_y]->second_height + m_height_sencond_high_threshold){  // 如果在桥洞里就设置障碍物
            GridNodeMap[idx_x][idx_y]->second_local_occupancy = true;  //桥下占用
        }else if(coord_z > GridNodeMap[idx_x][idx_y]->height + m_height_threshold && coord_z < GridNodeMap[idx_x][idx_y]->height + 0.3){  // 如果在桥上则在l_data中设置障碍物
            l_data[idx_x * GLY_SIZE + idx_y] = 1; // 桥上占用，在扫到桥洞前沿顶的时候膨胀可能会导致膨胀到外边，外边也是认为桥洞区域因此需要严格设置顶高的阈值(目前0.4)
        }
    }else{
        /* 将障碍物映射为容器里值为1的元素,根据这里的值做raycast process */
        l_data[idx_x * GLY_SIZE + idx_y] = 1;
    }
}
bool GlobalMap::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z, bool second_height) const
{  // 桥洞区域判断是否被占用需要特意处理
    if(second_height){
        return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
                (data[idx_x * GLY_SIZE + idx_y] == 1 || GridNodeMap[idx_x][idx_y]->second_local_occupancy == true));
    }
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 &&
            (data[idx_x * GLY_SIZE + idx_y] == 1 || l_data[idx_x * GLY_SIZE + idx_y] == 1));
}

bool GlobalMap::isOccupied(const Eigen::Vector3i &index, bool second_height) const {
    return isOccupied(index(0), index(1), index(2), second_height);
}

bool GlobalMap::isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const {
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
         (l_data[idx_x * GLY_SIZE + idx_y] == 1));
}

bool GlobalMap::isLocalOccupied(const Eigen::Vector3i &index) const {
    return isLocalOccupied(index(0), index(1), index(2));
}

bool GlobalMap::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const {
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
        (data[idx_x * GLY_SIZE + idx_y] < 1 && l_data[idx_x * GLY_SIZE + idx_y] < 1));
}

bool GlobalMap::isFree(const Eigen::Vector3i &index) const {
    return isFree(index(0), index(1), index(2));
}
