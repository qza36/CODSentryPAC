# CODSentryPAC - 轨迹生成系统

## 项目简介

基于 ROS2 的机器人轨迹生成系统，采用多层次路径规划架构，实现从起点到终点的平滑、安全的运动轨迹生成。适用于低速差速履带机器人。

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    ReplanFSM (状态机)                        │
│  INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ ⇄ REPLAN    │
└─────────────────────────────────────────────────────────────┘
                            │
┌─────────────────────────────────────────────────────────────┐
│                    planner_manager (规划管理器)              │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│ TopoSearcher │    │ AstarFinder  │    │  GlobalMap   │
│  (拓扑搜索)   │    │  (A*优化)    │    │  (地图管理)   │
└──────────────┘    └──────────────┘    └──────────────┘
        │                   │
        └───────────────────┘
                │
                ▼
        ┌──────────────┐
        │   Smoother   │
        │  (路径平滑)   │
        └──────────────┘
                │
                ▼
        ┌──────────────┐
        │Refenecesmooth│
        │(参考轨迹生成) │
        └──────────────┘
                │
                ▼
        ┌──────────────┐
        │ /trajectory  │
        │ (轨迹输出)    │
        └──────────────┘
```

## ROS2 接口

### 订阅话题

| 话题名 | 消息类型 | 功能 |
|--------|---------|------|
| `/odom` | `nav_msgs/Odometry` | 机器人位姿 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 目标点 |
| `/waypoints` | `nav_msgs/Path` | 路径点序列 |
| `/replan_flag` | `std_msgs/Bool` | 外部重规划触发 |

### 发布话题

| 话题名 | 消息类型 | 功能 |
|--------|---------|------|
| `/trajectory` | `TrajectoryPoly` | 多项式轨迹系数 |
| `/target_result` | `geometry_msgs/Point` | 当前目标点 |
| RViz Markers | `visualization_msgs/Marker` | 路径可视化 |

### 自定义消息 `TrajectoryPoly.msg`

```
builtin_interfaces/Time start_time  # 轨迹起始时间
uint8 motion_mode                   # 运动模式
float32[] coef_x                    # X方向多项式系数
float32[] coef_y                    # Y方向多项式系数
float32[] duration                  # 每段时间长度
```

轨迹表示：每段用三次多项式 `p(t) = c0 + c1*t + c2*t² + c3*t³`

## 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| 状态机 | `replan_fsm.hpp/cpp` | 规划状态管理、重规划触发 |
| 规划管理器 | `plannerManager.hpp/cpp` | 统一管理路径规划流程 |
| 地图管理 | `GridMap.hpp/cpp` | 3D 网格地图、障碍物检测 |
| 拓扑搜索 | `TopoSearch.hpp/cpp` | 基于 PRM 的多路径搜索 |
| A* 搜索 | `Astar_searcher.hpp/cpp` | 路径剪枝和碰撞检测 |
| 路径平滑 | `path_smooth.hpp/cpp` | L-BFGS 样条优化 |
| 参考轨迹 | `reference_path.hpp/cpp` | 三次样条时间参数化 |
| 可视化 | `visualization_utils.hpp/cpp` | RViz 可视化工具 |

## 路径生成流程

```
目标点 → 拓扑搜索(PRM) → A*剪枝 → L-BFGS平滑 → 三次样条轨迹 → 发布
```

### 第一层：拓扑路径搜索
- 算法：PRM (Probabilistic Roadmap)
- 输出：初始路径点序列

### 第二层：A* 路径优化
- 算法：贪心剪枝（可见性检查）
- 输出：简化后的路径

### 第三层：路径平滑
- 算法：L-BFGS 优化
- 代价函数：平滑能量 + 障碍物惩罚
- 输出：平滑路径

### 第四层：参考轨迹生成
- 时间分配：梯形速度规划（地形自适应）
- 插值：三次样条（带状矩阵求解）
- 输出：多项式轨迹系数

## 编译和运行

### 依赖项

- ROS2 (Humble/Foxy)
- Eigen3
- PCL (Point Cloud Library)
- OpenCV

### 编译

```bash
cd /path/to/workspace
colcon build --packages-select trajectory_generation
source install/setup.bash
```

### RViz 测试

```bash
ros2 launch trajectory_generation test.launch.py
```

在 RViz 中：
1. 使用工具栏的 **"2D Goal Pose"** 按钮
2. 在地图上点击设置目标点
3. 观察规划结果可视化

注意：需要先配置 `config/planner_config.yaml` 中的地图路径。

### 参数配置

编辑 `config/planner_config.yaml`：

```yaml
trajectory_generator:
  ros__parameters:
    # 地图文件路径
    occ_map_path: "/path/to/occ_map.png"

    # 动力学参数
    reference_v_max: 2.0        # 最大速度 (m/s)
    reference_a_max: 3.0        # 最大加速度 (m/s^2)
    reference_desire_speed: 1.5 # 期望速度 (m/s)

    # FSM 参数
    fsm:
      replan_thresh: 1.0        # 重规划距离阈值 (m)
      no_replan_thresh: 1.5     # 接近终点不重规划阈值 (m)
```

## 代码结构

```
trajectory_generation/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── planner_config.yaml     # 规划器参数
│   └── test.rviz               # RViz 配置
├── launch/
│   └── test.launch.py          # 测试启动文件
├── msg/
│   └── TrajectoryPoly.msg      # 轨迹消息定义
├── include/trajectory_generation/
│   ├── replan_fsm.hpp          # 状态机
│   ├── plannerManager.hpp      # 规划管理器
│   ├── GridMap.hpp             # 地图管理
│   ├── GridNode.hpp            # 网格节点
│   ├── TopoSearch.hpp          # 拓扑搜索
│   ├── Astar_searcher.hpp      # A* 搜索
│   ├── path_smooth.hpp         # 路径平滑
│   ├── reference_path.hpp      # 参考轨迹
│   ├── planner_config.hpp      # 配置结构
│   ├── visualization_utils.hpp # 可视化工具
│   └── root_solver/
│       ├── cubic_spline.hpp    # 三次样条
│       └── lbfgs.hpp           # L-BFGS 优化
├── src/
│   ├── trajectory_generator_node.cpp
│   ├── replan_fsm.cpp
│   ├── plannerManager.cpp
│   ├── planner_config.cpp
│   ├── GridMap.cpp
│   ├── TopoSearch.cpp
│   ├── Astar_searcher.cpp
│   ├── path_smooth.cpp
│   ├── reference_path.cpp
│   ├── visualization_utils.cpp
│   └── fake_odom_node.cpp      # 测试用假里程计
└── utils/
    └── getparm_utils.hpp
```

## 状态机

```
┌──────┐    odom received    ┌─────────────┐
│ INIT │ ─────────────────→  │ WAIT_TARGET │
└──────┘                     └─────────────┘
                                    │
                            target received
                                    ↓
                            ┌─────────────┐
                            │ GEN_NEW_TRAJ│ ←─────────────┐
                            └─────────────┘               │
                                    │                     │
                            planning success              │
                                    ↓                     │
┌────────────────┐         ┌─────────────┐         ┌─────────────┐
│ EMERGENCY_STOP │ ←────── │  EXEC_TRAJ  │ ←─────→ │ REPLAN_TRAJ │
└────────────────┘         └─────────────┘         └─────────────┘
    collision                     │
                           goal reached
                                  ↓
                          ┌─────────────┐
                          │ WAIT_TARGET │
                          └─────────────┘
```

## 限制

- 需要先验地图（PNG 格式）
- 不支持实时激光雷达建图

## 后续开发

- [ ] 集成 Nav2 支持实时建图
- [ ] 添加 OCS2 轨迹跟踪控制器
- [ ] 支持动态障碍物避障
