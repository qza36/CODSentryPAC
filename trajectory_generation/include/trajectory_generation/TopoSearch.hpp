/**
 * @file TopoSearch.hpp
 * @brief 拓扑路径搜索器 - 基于PRM的路径规划
 * @author zzt
 * @date 2023-10-22
 */

#ifndef TRAJECTORY_GENERATION_TOPOSEARCH_H
#define TRAJECTORY_GENERATION_TOPOSEARCH_H

#include <random>
#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <utility>
#include <vector>
#include "trajectory_generation/GridMap.hpp"

/**
 * @brief PRM图节点
 *
 * 节点类型:
 * - Guard: 守卫节点，采样空间中的关键点
 * - Connector: 连接节点，连接两个守卫节点
 */
class GraphNode
{
public:
    enum NODE_TYPE {
        Guard = 1,      // 守卫节点
        Connector = 2   // 连接节点
    };

    enum NODE_STATE {
        NEW = 1,    // 新建
        CLOSE = 2,  // 已关闭
        OPEN = 3    // 开放中
    };

    GraphNode() {}
    GraphNode(Eigen::Vector3d position, NODE_TYPE type, int id) {
        pos = position;
        type_ = type;
        state_ = NEW;
        m_id = id;
    }
    ~GraphNode() {}

    std::vector<std::shared_ptr<GraphNode>> neighbors;  // 邻居节点
    std::vector<std::shared_ptr<GraphNode>> neighbors_but_noconnected;  // 可见但未连接的节点
    NODE_TYPE type_;
    NODE_STATE state_;

    Eigen::Vector3d pos;  // 节点位置
    int m_id;             // 节点ID

    typedef std::shared_ptr<GraphNode> Ptr;
};

/**
 * @brief 拓扑路径搜索器
 *
 * 算法流程:
 * 1. 在椭圆采样空间内随机采样
 * 2. 建立守卫节点(Guard)和连接节点(Connector)
 * 3. 构建PRM图
 * 4. 使用Dijkstra搜索最短路径
 * 5. DFS搜索多条候选路径
 */
class TopoSearcher
{
public:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<GlobalMap> global_map;

    // ==================== 随机采样器 ====================
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;

    // ==================== 采样空间变换 ====================
    Eigen::Vector3d m_sample_r;       // 椭圆采样半径
    Eigen::Vector3d m_translation;    // 采样空间平移
    Eigen::Matrix3d m_rotation;       // 采样空间旋转
    Eigen::Matrix3d m_rotation_inv;   // 旋转逆矩阵
    Eigen::Vector3d m_sample_inflate; // 采样膨胀量

    // ==================== 搜索结果 ====================
    std::vector<GraphNode::Ptr> m_graph;  // PRM图节点
    std::vector<std::vector<Eigen::Vector3d>> raw_paths;    // 原始路径集合
    std::vector<std::vector<Eigen::Vector3d>> final_paths;  // 最终路径集合(k最优)
    std::vector<Eigen::Vector3d> min_path;  // 最短路径

    std::multimap<double, std::vector<Eigen::Vector3d>> distanceSet;  // 按距离排序的路径集合

    // ==================== 参数 ====================
    double max_sample_time;  // 最大采样时间
    int max_sample_num;      // 最大采样数量
    double m_resolution;     // 采样分辨率

    // ==================== 公共接口 ====================
    void init(std::shared_ptr<GlobalMap> &_global_map);

    /**
     * @brief 创建全局拓扑图
     * @param start 起点
     * @param end 终点
     */
    void createGraph(Eigen::Vector3d start, Eigen::Vector3d end);

    /**
     * @brief 创建局部拓扑图(用于重规划)
     * @param start 起点
     * @param end 终点
     * @param attack_target 是否为攻击目标模式
     */
    void createLocalGraph(Eigen::Vector3d start, Eigen::Vector3d end, bool attack_target = false);

    /**
     * @brief 搜索可行路径
     * @param node_id 起始节点ID
     * @return 路径集合
     */
    std::vector<std::vector<Eigen::Vector3d>> searchPaths(int node_id = 1);

    /**
     * @brief Dijkstra最短路径搜索
     */
    void DijkstraSearch(int node_id);

    // ==================== 采样函数 ====================
    Eigen::Vector3d getSample();       // 全局采样
    Eigen::Vector3d getLocalSample();  // 局部采样

    // ==================== 可见性检查 ====================
    /**
     * @brief 两点间直线可见性检查
     * @param p1 起点
     * @param p2 终点
     * @param thresh 采样阈值
     * @param pc [out] 碰撞点
     * @param caster_id 射线ID
     * @return true=可见
     */
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                   double thresh, Eigen::Vector3d& pc, int caster_id = 0);

    /**
     * @brief 高度可行性检查
     * @param p1 起点
     * @param p2 终点
     * @param direction [out] 方向(上坡/下坡)
     * @param step 采样步长
     * @param thresh 高度阈值
     * @return true=可行
     */
    bool heightFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                        int &direction, double step = 0.05, double thresh = 0.10);

    bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);

    // ==================== 图构建辅助函数 ====================
    std::vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt, double &dis_temp, int &exist_unvisiual);
    std::vector<GraphNode::Ptr> findMinVisibGuard(Eigen::Vector3d pt, int &exist_unvisiual);
    bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);
    bool searchOnePointPath(Eigen::Vector3d end);
    void pruneGraph();

    // ==================== 路径处理 ====================
    void depthFirstSearch(std::vector<GraphNode::Ptr>& vis);
    void checkDistanceFinalPath();
    double pathLength(const std::vector<Eigen::Vector3d>& path);
    std::vector<Eigen::Vector3d> discretizePath(std::vector<Eigen::Vector3d> path, int pt_num);
    bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1,
                      const std::vector<Eigen::Vector3d>& path2, double thresh);

    // ==================== 方向检查 ====================
    int checkSearchDirection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);
    void setSearchDirection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt,
                            int &node_id, int direction);
    void checkHeightFeasible(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id);
};

#endif //TRAJECTORY_GENERATION_TOPOSEARCH_H
