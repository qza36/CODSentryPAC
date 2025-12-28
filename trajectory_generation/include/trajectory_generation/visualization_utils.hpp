/**
 * @file visualization_utils.hpp
 * @brief 可视化工具 - 用于RViz调试和展示规划结果
 */

#ifndef VISUALIZATION_UTILS_HPP
#define VISUALIZATION_UTILS_HPP

#include <Eigen/Eigen>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "trajectory_generation/TopoSearch.hpp"
#include "trajectory_generation/GridMap.hpp"

/**
 * @brief 可视化工具类
 *
 * 功能:
 * - A*路径可视化
 * - 优化路径可视化
 * - 参考轨迹可视化
 * - 拓扑图可视化
 * - 当前位置/目标位置可视化
 * - 栅格地图可视化
 */
class Visualization
{
public:
    Visualization() = default;
    ~Visualization() = default;

    /**
     * @brief 初始化可视化发布者
     * @param node ROS2节点指针
     */
    void init(rclcpp::Node::SharedPtr node);

    // ==================== 路径可视化 ====================
    void visAstarPath(const std::vector<Eigen::Vector3d>& nodes);
    void visFinalPath(const std::vector<Eigen::Vector3d>& nodes);
    void visOptimizedPath(const std::vector<Eigen::Vector2d>& nodes);
    void visReferencePath(const std::vector<Eigen::Vector3d>& nodes);

    // ==================== 位置可视化 ====================
    void visCurPosition(const Eigen::Vector3d& cur_pt);
    void visTargetPosition(const Eigen::Vector3d& target_pt);

    // ==================== 拓扑图可视化 ====================
    void visTopoPointGuard(const std::vector<GraphNode::Ptr>& graph);
    void visTopoPointConnection(const std::vector<GraphNode::Ptr>& graph);
    void visTopoPath(const std::vector<std::vector<Eigen::Vector3d>>& paths);

    // ==================== 障碍物可视化 ====================
    void visObstacles(const std::vector<std::vector<Eigen::Vector3d>>& obs);

    // ==================== 地图可视化 ====================
    void visGridMap(const std::shared_ptr<GlobalMap>& global_map);

private:
    rclcpp::Node::SharedPtr node_;
    std::string frame_id_ = "world";

    // 发布者
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr astar_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_line_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cur_position_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_position_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_guard_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_connection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_line_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_path_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_pub_;

    // 辅助函数
    visualization_msgs::msg::Marker createMarker(
        const std::string& ns,
        int type,
        const std::array<float, 4>& color,
        const std::array<float, 3>& scale,
        int id = 0);
};

#endif
