/**
 * @file visualization_utils.cpp
 * @brief 可视化工具实现
 */

#include "trajectory_generation/visualization_utils.hpp"

void Visualization::init(rclcpp::Node::SharedPtr node)
{
    node_ = node;

    // 创建发布者
    astar_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("astar_path_vis", 1);
    final_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("final_path_vis", 1);
    final_line_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("final_line_vis", 1);
    optimized_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("optimized_path_vis", 1);
    reference_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("reference_path_vis", 1);
    cur_position_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("cur_position_vis", 1);
    target_position_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("target_vis", 1);
    topo_guard_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("topo_guard_vis", 1);
    topo_connection_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("topo_connection_vis", 1);
    topo_line_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("topo_line_vis", 1);
    topo_path_point_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("topo_path_point_vis", 1);
    topo_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("topo_path_vis", 1);
    obs_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("obs_vis", 1);
}

visualization_msgs::msg::Marker Visualization::createMarker(
    const std::string& ns,
    int type,
    const std::array<float, 4>& color,
    const std::array<float, 3>& scale,
    int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = id;

    marker.pose.orientation.w = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];

    return marker;
}

void Visualization::visAstarPath(const std::vector<Eigen::Vector3d>& nodes)
{
    auto marker = createMarker("astar_path",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {0.0f, 1.0f, 0.0f, 0.4f},  // 绿色
        {0.1f, 0.1f, 0.1f});

    for (const auto& node : nodes) {
        geometry_msgs::msg::Point pt;
        pt.x = node.x();
        pt.y = node.y();
        pt.z = node.z();
        marker.points.push_back(pt);
    }

    astar_path_pub_->publish(marker);
}

void Visualization::visFinalPath(const std::vector<Eigen::Vector3d>& nodes)
{
    // 路径点
    auto marker = createMarker("final_path",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {0.7f, 0.0f, 0.4f, 0.8f},  // 紫色
        {0.1f, 0.1f, 1.0f});

    // 连线
    auto line = createMarker("final_path",
        visualization_msgs::msg::Marker::LINE_LIST,
        {0.0f, 1.0f, 1.0f, 1.0f},  // 青色
        {0.04f, 0.04f, 0.04f}, 1);

    for (size_t i = 0; i < nodes.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = nodes[i].x();
        pt.y = nodes[i].y();
        pt.z = nodes[i].z();
        marker.points.push_back(pt);

        if (i < nodes.size() - 1) {
            geometry_msgs::msg::Point next_pt;
            next_pt.x = nodes[i + 1].x();
            next_pt.y = nodes[i + 1].y();
            next_pt.z = nodes[i + 1].z();
            line.points.push_back(pt);
            line.points.push_back(next_pt);
        }
    }

    final_path_pub_->publish(marker);
    final_line_pub_->publish(line);
}

void Visualization::visOptimizedPath(const std::vector<Eigen::Vector2d>& nodes)
{
    auto marker = createMarker("optimized_path",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {0.0f, 0.6f, 0.6f, 1.0f},  // 青色
        {0.1f, 0.1f, 0.1f});

    for (const auto& node : nodes) {
        geometry_msgs::msg::Point pt;
        pt.x = node.x();
        pt.y = node.y();
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    optimized_path_pub_->publish(marker);
}

void Visualization::visReferencePath(const std::vector<Eigen::Vector3d>& nodes)
{
    auto marker = createMarker("reference_path",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {1.0f, 1.0f, 0.0f, 1.0f},  // 黄色
        {0.04f, 0.04f, 0.04f});

    for (const auto& node : nodes) {
        geometry_msgs::msg::Point pt;
        pt.x = node.x();
        pt.y = node.y();
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    reference_path_pub_->publish(marker);
}

void Visualization::visCurPosition(const Eigen::Vector3d& cur_pt)
{
    auto marker = createMarker("cur_position",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {0.0f, 1.0f, 1.0f, 1.0f},  // 青色
        {0.1f, 0.1f, 2.0f});

    geometry_msgs::msg::Point pt;
    pt.x = cur_pt.x();
    pt.y = cur_pt.y();
    pt.z = cur_pt.z();
    marker.points.push_back(pt);

    cur_position_pub_->publish(marker);
}

void Visualization::visTargetPosition(const Eigen::Vector3d& target_pt)
{
    auto marker = createMarker("target_position",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {1.0f, 0.0f, 1.0f, 1.0f},  // 紫色
        {0.1f, 0.1f, 2.0f});

    geometry_msgs::msg::Point pt;
    pt.x = target_pt.x();
    pt.y = target_pt.y();
    pt.z = target_pt.z();
    marker.points.push_back(pt);

    target_position_pub_->publish(marker);
}

void Visualization::visTopoPointGuard(const std::vector<GraphNode::Ptr>& graph)
{
    auto marker = createMarker("topo_guard",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {1.0f, 0.0f, 0.0f, 1.0f},  // 红色
        {0.05f, 0.05f, 0.5f});

    for (const auto& node : graph) {
        if (node->type_ != GraphNode::Guard) continue;

        geometry_msgs::msg::Point pt;
        pt.x = node->pos.x();
        pt.y = node->pos.y();
        pt.z = node->pos.z() * 2;
        marker.points.push_back(pt);
    }

    topo_guard_pub_->publish(marker);
}

void Visualization::visTopoPointConnection(const std::vector<GraphNode::Ptr>& graph)
{
    auto marker = createMarker("topo_connection",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {0.0f, 1.0f, 0.0f, 1.0f},  // 绿色
        {0.03f, 0.03f, 0.5f});

    auto line = createMarker("topo_connection",
        visualization_msgs::msg::Marker::LINE_LIST,
        {1.0f, 0.0f, 0.0f, 1.0f},  // 红色
        {0.02f, 0.02f, 0.02f}, 1);

    for (const auto& node : graph) {
        if (node->type_ != GraphNode::Connector) continue;

        geometry_msgs::msg::Point pt;
        pt.x = node->pos.x();
        pt.y = node->pos.y();
        pt.z = node->pos.z() * 2;
        marker.points.push_back(pt);

        // 绘制连接线
        if (node->neighbors.size() >= 2) {
            for (const auto& neighbor : node->neighbors) {
                geometry_msgs::msg::Point neigh_pt;
                neigh_pt.x = neighbor->pos.x();
                neigh_pt.y = neighbor->pos.y();
                neigh_pt.z = neighbor->pos.z() * 2;
                line.points.push_back(pt);
                line.points.push_back(neigh_pt);
            }
        }
    }

    topo_connection_pub_->publish(marker);
    topo_line_pub_->publish(line);
}

void Visualization::visTopoPath(const std::vector<std::vector<Eigen::Vector3d>>& paths)
{
    auto marker = createMarker("topo_path",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {0.0f, 0.0f, 1.0f, 1.0f},  // 蓝色
        {0.05f, 0.05f, 0.5f});

    auto line = createMarker("topo_path",
        visualization_msgs::msg::Marker::LINE_LIST,
        {0.0f, 1.0f, 0.0f, 1.0f},  // 绿色
        {0.05f, 0.05f, 0.05f}, 1);

    for (const auto& path : paths) {
        for (size_t i = 0; i < path.size(); ++i) {
            geometry_msgs::msg::Point pt;
            pt.x = path[i].x();
            pt.y = path[i].y();
            pt.z = path[i].z() * 2;
            marker.points.push_back(pt);

            if (i < path.size() - 1) {
                geometry_msgs::msg::Point next_pt;
                next_pt.x = path[i + 1].x();
                next_pt.y = path[i + 1].y();
                next_pt.z = path[i + 1].z();
                line.points.push_back(pt);
                line.points.push_back(next_pt);
            }
        }
    }

    topo_path_point_pub_->publish(marker);
    topo_path_pub_->publish(line);
}

void Visualization::visObstacles(const std::vector<std::vector<Eigen::Vector3d>>& obs)
{
    auto marker = createMarker("obstacles",
        visualization_msgs::msg::Marker::CUBE_LIST,
        {1.0f, 0.0f, 0.0f, 1.0f},  // 红色
        {0.1f, 0.1f, 1.0f});

    if (!obs.empty() && obs.size() >= 3) {
        for (const auto& pt : obs[obs.size() - 3]) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = 0.0;
            marker.points.push_back(p);
        }
    }

    obs_pub_->publish(marker);
}
