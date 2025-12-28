/**
 * @file path_smooth.hpp
 * @brief 路径平滑器 - 基于L-BFGS优化的路径平滑
 */

#ifndef PATH_SMOOTH_HPP
#define PATH_SMOOTH_HPP

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
#include "root_solver/cubic_spline.hpp"
#include "root_solver/lbfgs.hpp"
#include "GridMap.hpp"

/**
 * @brief 路径平滑器
 *
 * 功能:
 * - 对拓扑路径进行平滑优化
 * - 使用L-BFGS优化器最小化路径能量
 * - 考虑障碍物排斥力
 * - 生成三次样条曲线
 *
 * 优化目标: E = 平滑能量 + 障碍物势场能量
 */
class Smoother
{
public:
    Smoother() {};
    ~Smoother();

    std::shared_ptr<GlobalMap> global_map;

    // ==================== 障碍物数据 ====================
    std::vector<Eigen::Vector3d> obs_coord;  // 障碍物坐标
    std::vector<std::vector<Eigen::Vector3d>> allobs;  // 所有障碍物
    std::vector<double> mid_distance;  // 中间距离
    std::vector<double> weighs;        // 权重

    // ==================== 路径数据 ====================
    Eigen::Matrix2Xd pathInPs;  // 路径控制点 (2 x N)
    std::vector<double> m_trapezoidal_time;  // 梯形时间分配
    double desire_veloity;  // 期望速度

    bool init_obs = false;  // 障碍物初始化标志
    bool init_vel = false;  // 速度初始化标志

    // ==================== 公共接口 ====================
    void setGlobalMap(std::shared_ptr<GlobalMap> &_global_map);

    /**
     * @brief 初始化平滑器
     * @param global_path 全局路径
     * @param start_vel 起始速度
     * @param desire_speed 期望速度
     */
    void init(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel, double desire_speed);

    /**
     * @brief 执行路径平滑
     */
    void smoothPath();

    /**
     * @brief 路径采样
     * @param global_path 原始路径
     * @param start_vel 起始速度
     */
    void pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel);

    /**
     * @brief 路径重采样
     */
    void pathResample();

    /**
     * @brief 获取平滑后的路径
     */
    std::vector<Eigen::Vector2d> getPath();

    /**
     * @brief 获取采样路径
     */
    std::vector<Eigen::Vector2d> getSamplePath();

    // ==================== 障碍物处理 ====================
    void getObsEdge(Eigen::Vector2d xcur);

    /**
     * @brief 计算障碍物排斥力
     * @param idx 点索引
     * @param xcur 当前点
     * @param nearest_cost [out] 最近障碍物代价
     * @return 排斥力梯度
     */
    Eigen::Vector2d obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost);

    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isFree(const Eigen::Vector3i &index) const;

    bool getObsPosition(double start_x, double start_y, double start_z,
                        double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt);

    void getGuidePath(Eigen::Vector2d start_vel, double radius = 0.5);

    /**
     * @brief L-BFGS优化目标函数
     * @param ptr Smoother指针
     * @param x 优化变量
     * @param g [out] 梯度
     * @return 代价值
     */
    static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);

private:
    float wObstacle = 1e5;  // 障碍物权重
    int pieceN;             // 路径段数

    Eigen::Vector2d headP;  // 起点
    Eigen::Vector2d tailP;  // 终点

    CubicSpline cubSpline;  // 三次样条
    lbfgs::lbfgs_parameter lbfgs_params;  // L-BFGS参数

    std::vector<Eigen::Vector2d> path;       // 采样初始轨迹
    std::vector<Eigen::Vector2d> finalpath;  // 优化后轨迹

    // 时间变换函数(用于优化)
    static void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);
    static void backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau);
    static void backwardGradT(const Eigen::VectorXd &tau, Eigen::VectorXd &gradT, Eigen::VectorXd &gradTau);
};

#endif
