/**
 * @file reference_path.hpp
 * @brief 参考轨迹生成器 - 三次样条轨迹生成
 */

#ifndef REFERENCE_PATH_HPP
#define REFERENCE_PATH_HPP

#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "GridMap.hpp"

/**
 * @brief 带状矩阵求解器
 *
 * 用于求解带状线性方程组 Ax=b
 * A 是 N*N 的带状矩阵，lower band为lowerBw，upper band为upperBw
 * 时间复杂度: O(N)
 */
class BandedSystem
{
public:
    /**
     * @brief 创建带状矩阵
     * @param n 矩阵维度
     * @param p 下带宽
     * @param q 上带宽
     */
    inline void create(const int &n, const int &p, const int &q) {
        destroy();
        N = n;
        lowerBw = p;
        upperBw = q;
        int actualSize = N * (lowerBw + upperBw + 1);
        ptrData = new double[actualSize];
        std::fill_n(ptrData, actualSize, 0.0);
    }

    inline void destroy() {
        if (ptrData != nullptr) {
            delete[] ptrData;
            ptrData = nullptr;
        }
    }

    inline void operator=(const BandedSystem &bs) {
        ptrData = nullptr;
        create(bs.N, bs.lowerBw, bs.upperBw);
        memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
    }

private:
    int N;
    int lowerBw;
    int upperBw;
    double *ptrData = nullptr;

public:
    inline void reset(void) {
        std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    }

    // 矩阵元素访问 (按"Matrix Computation"建议的存储方式)
    inline const double &operator()(const int &i, const int &j) const {
        return ptrData[(i - j + upperBw) * N + j];
    }

    inline double &operator()(const int &i, const int &j) {
        return ptrData[(i - j + upperBw) * N + j];
    }

    /**
     * @brief LU分解 (无主元选取，追求效率)
     */
    inline void factorizeLU() {
        int iM, jM;
        double cVl;
        for (int k = 0; k <= N - 2; k++) {
            iM = std::min(k + lowerBw, N - 1);
            cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; i++) {
                if (operator()(i, k) != 0.0) {
                    operator()(i, k) /= cVl;
                }
            }
            jM = std::min(k + upperBw, N - 1);
            for (int j = k + 1; j <= jM; j++) {
                cVl = operator()(k, j);
                if (cVl != 0.0) {
                    for (int i = k + 1; i <= iM; i++) {
                        if (operator()(i, k) != 0.0) {
                            operator()(i, j) -= operator()(i, k) * cVl;
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief 求解 Ax=b，结果存储在b中
     * @param b 输入为右端项，输出为解
     */
    inline void solve(Eigen::MatrixXd &b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            iM = std::min(j + lowerBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            b.row(j) /= operator()(j, j);
            iM = std::max(0, j - upperBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
    }

    /**
     * @brief 求解 A^T x=b
     */
    inline void solveAdj(Eigen::MatrixXd &b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            iM = std::max(0, j - lowerBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


/**
 * @brief 参考轨迹生成器
 *
 * 功能:
 * - 生成时间最优的参考轨迹
 * - 三次样条插值
 * - 梯形速度规划
 * - 地形自适应速度调整(坡道、桥洞)
 *
 * 轨迹表示: 分段三次多项式
 */
class Refenecesmooth
{
public:
    Refenecesmooth() {};
    ~Refenecesmooth();

    // ==================== 轨迹数据 ====================
    std::vector<double> m_trapezoidal_time;        // 梯形时间分配
    std::vector<Eigen::Vector3d> reference_path;   // 参考路径点
    std::vector<Eigen::Vector3d> reference_velocity;// 参考速度

    Eigen::MatrixXd m_polyMatrix_x;  // X轴多项式系数矩阵
    Eigen::MatrixXd m_polyMatrix_y;  // Y轴多项式系数矩阵
    BandedSystem m_bandedMatrix;     // 带状矩阵求解器
    Eigen::MatrixXd b;               // 右端项
    Eigen::MatrixXd b2;
    Eigen::Vector3d state_vel;       // 状态速度

    // ==================== 动力学参数 ====================
    double max_accleration = 6.4;   // 最大加速度 (m/s^2)
    double max_velocity = 4.0;      // 最大速度 (m/s)
    double desire_veloity = 3.6;    // 期望速度 (m/s)
    double dt = 0.05;               // 时间步长 (s)
    double traj_length = 0.0;       // 轨迹总长度

    // ==================== 地形系数 ====================
    // TODO: 建议移到参数表
    double slope_coeff = 1.8;       // 坡道系数(正常模式)
    double slope_coeff_xtl = 1.4;   // 坡道系数(小陀螺模式)
    double slope_coeff_max = 1.0;
    double slope_coeff_xtl_max = 0.6;

    double bridge_coeff = 1.8;      // 桥洞系数(正常模式)
    double bridge_coeff_xtl = 1.6;  // 桥洞系数(小陀螺模式)

    bool isxtl = false;  // 小陀螺模式标志

    std::shared_ptr<GlobalMap> global_map;

    // ==================== 公共接口 ====================
    void init(std::shared_ptr<GlobalMap> &_global_map);

    /**
     * @brief 设置全局路径并生成轨迹
     * @param velocity 初始速度
     * @param global_path 全局路径
     * @param reference_amax 最大加速度
     * @param desire_speed 期望速度
     * @param xtl 是否为小陀螺模式
     */
    void setGlobalPath(Eigen::Vector3d velocity,
                       std::vector<Eigen::Vector2d> &global_path,
                       double reference_amax,
                       double desire_speed,
                       bool xtl);

    /**
     * @brief 计算梯形时间分配
     */
    void solveTrapezoidalTime();

    /**
     * @brief 求解多项式系数矩阵
     */
    void solvePolyMatrix();

    /**
     * @brief 检查轨迹动力学可行性
     */
    bool checkfeasible();

    // ==================== 轨迹查询 ====================
    Eigen::Vector2d getRefPose();
    void getRefVel();
    Eigen::Vector2d getRefAcc();

    /**
     * @brief 获取参考轨迹
     * @param ref_trajectory [out] 轨迹点序列
     * @param times [out] 时间序列
     */
    void getRefTrajectory(std::vector<Eigen::Vector3d> &ref_trajectory, std::vector<double> &times);

    /**
     * @brief 获取跟踪轨迹(用于MPC)
     */
    void getTrackingTraj(int step_time, int planning_horizon, Eigen::Vector3d state,
                         std::vector<Eigen::Vector3d> &ref_trajectory,
                         std::vector<double> &ref_phi, std::vector<double> &ref_speed);

    void getSegmentIndex(double time, int &segment_index, double &total_time);

    void reset(const Eigen::Matrix3d &headState, const Eigen::Matrix3d &tailState, const int &pieceNum);
    void reset(const int &pieceNum);

private:
    int N;
    Eigen::Matrix3d headPVA;  // 起点位置、速度、加速度
    Eigen::Matrix3d tailPVA;  // 终点位置、速度、加速度
    std::vector<Eigen::Vector2d> m_global_path;
};

#endif
