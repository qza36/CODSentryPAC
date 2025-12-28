#include "trajectory_generation/planner_config.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/plannerManager.hpp"
#include "trajectory_generation/Astar_searcher.hpp"
#include "trajectory_generation/TopoSearch.hpp"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("trajectory_generation.planner_config");

// 参数校验辅助函数
bool validateMapConfig(const planner_manager::PlannerConfig& config) {
    bool valid = true;

    // 检查地图尺寸
    if (config.map.map_grid_size(0) <= 0 || config.map.map_grid_size(1) <= 0) {
        RCLCPP_ERROR(kLogger, "[Config] Invalid grid size: (%d, %d, %d) - must be positive!",
                     config.map.map_grid_size(0), config.map.map_grid_size(1), config.map.map_grid_size(2));
        valid = false;
    }

    // 检查分辨率
    if (config.map.map_resolution <= 0) {
        RCLCPP_ERROR(kLogger, "[Config] Invalid resolution: %.3f - must be positive!", config.map.map_resolution);
        valid = false;
    }

    // 检查地图文件路径
    if (config.map.occ_map_path.empty()) {
        RCLCPP_WARN(kLogger, "[Config] occ_map_path is empty - using default empty map");
    } else if (!std::filesystem::exists(config.map.occ_map_path)) {
        RCLCPP_ERROR(kLogger, "[Config] occ_map_path does not exist: %s", config.map.occ_map_path.c_str());
        valid = false;
    }

    if (!config.map.distance_map_path.empty() && !std::filesystem::exists(config.map.distance_map_path)) {
        RCLCPP_WARN(kLogger, "[Config] distance_map_path does not exist: %s", config.map.distance_map_path.c_str());
    }

    return valid;
}
}

void planner_manager::init(rclcpp::Node::SharedPtr node)
{
   // 节点校验
   if (!node) {
       RCLCPP_FATAL(kLogger, "[Manager Init] node is nullptr!");
       throw std::runtime_error("planner_manager::init() - node is nullptr");
   }

   RCLCPP_INFO(kLogger, "[Manager Init] Starting initialization...");

   PlannerConfig config;
   //地图参数
   apex_utils::get_param(node,"height_bias",config.map.height_bias,-0.5);
   apex_utils::get_param(node,"height_interval",config.map.height_interval,2.0);
   apex_utils::get_param(node,"height_threshold",config.map.height_threshold,0.1);
   apex_utils::get_param(node,"height_sencond_high_threshold",config.map.height_sencond_high_threshold,0.4);
   //map lower
   apex_utils::get_param(node,"map_lower_point_x",config.map.map_lower_point(0),0.0);
   apex_utils::get_param(node, "map_lower_point_y", config.map.map_lower_point(1), 0.0);
   apex_utils::get_param(node, "map_lower_point_z", config.map.map_lower_point(2), 0.0);
   //map upper
   apex_utils::get_param(node,"map_upper_point_x",config.map.map_upper_point(0),0.0);
   apex_utils::get_param(node, "map_upper_point_y", config.map.map_upper_point(1), 0.0);
   apex_utils::get_param(node, "map_upper_point_z", config.map.map_upper_point(2), 0.0);
   //gird_size
   apex_utils::get_param(node, "grid_max_id_x", config.map.map_grid_size(0), 0);
   apex_utils::get_param(node, "grid_max_id_y", config.map.map_grid_size(1), 0);
   apex_utils::get_param(node, "grid_max_id_z", config.map.map_grid_size(2), 0);
   apex_utils::get_param(node, "resolution", config.map.map_resolution, 0.0);
   //map_file_path
   apex_utils::get_param<std::string>(node,"occ_map_path",config.map.occ_map_path,"");
   apex_utils::get_param<std::string>(node,"bev_map_path",config.map.bev_map_path,"");
   apex_utils::get_param<std::string>(node,"distance_map_path",config.map.distance_map_path,"");
   apex_utils::get_param<std::string>(node,"topo_map_path",config.map.topo_map_path,"");


   //search
   apex_utils::get_param(node,"search_height_min",config.search.search_height_min,0.0);
   apex_utils::get_param(node,"search_height_max",config.search.search_height_max,0.5);
   apex_utils::get_param(node,"search_radius",config.search.search_radius,0.2);
   apex_utils::get_param(node,"robot_radius",config.search.robot_radius,0.3);
   apex_utils::get_param(node,"robot_radius_dash",config.search.robot_radius_dash,0.3);

   //dynamics
   apex_utils::get_param(node,"reference_v_max",config.dynamics.v_max,2.0);
   apex_utils::get_param(node,"reference_a_max",config.dynamics.a_max,3.0);
   apex_utils::get_param(node,"reference_w_max",config.dynamics.w_max,2.0);
   apex_utils::get_param(node,"reference_desire_speed",config.dynamics.desire_speed,1.5);

   // 打印配置信息
   RCLCPP_INFO(kLogger, "[Manager Init] Map config:");
   RCLCPP_INFO(kLogger, "  - occ_map_path: %s", config.map.occ_map_path.c_str());
   RCLCPP_INFO(kLogger, "  - bev_map_path: %s", config.map.bev_map_path.c_str());
   RCLCPP_INFO(kLogger, "  - distance_map_path: %s", config.map.distance_map_path.c_str());
   RCLCPP_INFO(kLogger, "  - grid_size: (%d, %d, %d)", config.map.map_grid_size(0), config.map.map_grid_size(1), config.map.map_grid_size(2));
   RCLCPP_INFO(kLogger, "  - resolution: %.3f", config.map.map_resolution);
   RCLCPP_INFO(kLogger, "  - map_bounds: (%.1f,%.1f) to (%.1f,%.1f)",
               config.map.map_lower_point(0), config.map.map_lower_point(1),
               config.map.map_upper_point(0), config.map.map_upper_point(1));

   // 参数校验
   if (!validateMapConfig(config)) {
       RCLCPP_FATAL(kLogger, "[Manager Init] Invalid configuration! Check parameters.");
       throw std::runtime_error("planner_manager::init() - invalid configuration");
   }

   m_eng = std::default_random_engine(m_rd());
   m_rand_pos = std::uniform_real_distribution<double>(-1.0, 1.0);

   sentryColor = teamColor::red;

   // 初始化各模块
   RCLCPP_INFO(kLogger, "[Manager Init] Initializing GlobalMap...");
   global_map.reset(new GlobalMap);
   global_map->initGridMap(node, config);

   RCLCPP_INFO(kLogger, "[Manager Init] Initializing AstarPathFinder...");
   astar_path_finder.reset(new AstarPathFinder);
   astar_path_finder->initGridMap(node, global_map);

   RCLCPP_INFO(kLogger, "[Manager Init] Initializing Smoother...");
   path_smoother.reset(new Smoother);
   path_smoother->setGlobalMap(global_map);

   RCLCPP_INFO(kLogger, "[Manager Init] Initializing ReferencePath...");
   reference_path.reset(new Refenecesmooth);
   reference_path->init(global_map);
   reference_path->max_accleration = config.dynamics.a_max;
   reference_path->max_velocity = config.dynamics.v_max;
   reference_path->desire_veloity = config.dynamics.desire_speed;

   RCLCPP_INFO(kLogger, "[Manager Init] Initializing TopoSearcher...");
   topo_prm.reset(new TopoSearcher);
   topo_prm->init(node, global_map);

   global_map->setRadiusDash(config.search.robot_radius_dash);

   // 设置全局动力学参数
   reference_v_max = config.dynamics.v_max;
   reference_a_max = config.dynamics.a_max;
   reference_w_max = config.dynamics.w_max;
   reference_desire_speed = config.dynamics.desire_speed;
   reference_desire_speedxtl = config.dynamics.desire_speed * 0.8;  // 巡逻模式速度稍低

   RCLCPP_INFO(kLogger, "[Manager Init] Dynamics: v_max=%.2f, a_max=%.2f, desire_speed=%.2f",
               reference_v_max, reference_a_max, reference_desire_speed);

   RCLCPP_INFO(kLogger, "[Manager Init] All modules initialized successfully!");
}