#include "trajectory_generation/planner_config.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/plannerManger.hpp"
#include "trajectory_generation/Astar_searcher.hpp"
#include "trajectory_generation/TopoSearch.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("trajectory_generation.planner_config");
}

void planner_manager::init(rclcpp::Node::SharedPtr node)
{
   PlannerConfig config;
   //地图参数
   apex_utils::get_param(node,"trajectory_generator.height_bias",config.map.height_bias,-0.5);
   apex_utils::get_param(node,"trajectory_generator.height_interval",config.map.height_interval,2.0);
   apex_utils::get_param(node,"trajectory_generator.height_threshold",config.map.height_threshold,0.1);
   apex_utils::get_param(node,"trajectory_generator.height_sencond_high_threshold",config.map.height_sencond_high_threshold,0.4);
   //map lower
   apex_utils::get_param(node,"trajectory_generator.map_lower_point_x",config.map.map_lower_point(0),0.0);
   apex_utils::get_param(node, "trajectory_generator.map_lower_point_y", config.map.map_lower_point(1), 0.0);
   apex_utils::get_param(node, "trajectory_generator.map_lower_point_z", config.map.map_lower_point(2), 0.0);
   //map upper
   apex_utils::get_param(node,"trajectory_generator.map_upper_point_x",config.map.map_upper_point(0),0.0);
   apex_utils::get_param(node, "trajectory_generator.map_upper_point_y", config.map.map_upper_point(1), 0.0);
   apex_utils::get_param(node, "trajectory_generator.map_upper_point_z", config.map.map_upper_point(2), 0.0);
   //gird_size
   apex_utils::get_param(node, "trajectory_generator.grid_max_id_x", config.map.map_grid_size(0), 0);
   apex_utils::get_param(node, "trajectory_generator.grid_max_id_y", config.map.map_grid_size(1), 0);
   apex_utils::get_param(node, "trajectory_generator.grid_max_id_z", config.map.map_grid_size(2), 0);
   apex_utils::get_param(node, "trajectory_generator.resolution", config.map.map_resolution, 0.0);
   //map_file_path
   apex_utils::get_param<std::string>(node,"trajectory_generator.occ_map_path",config.map.occ_map_path,"/home/cod-sentry/qza_ws/cod_planning/src/trajectory_generation/map/occ2024.png");
   apex_utils::get_param<std::string>(node,"trajectory_generator.bev_map_path",config.map.bev_map_path,"/path");
   apex_utils::get_param<std::string>(node,"trajectory_generator.distance_map_path",config.map.distance_map_path,"/path");
   apex_utils::get_param<std::string>(node,"trajectory_generator.topo_map_path",config.map.topo_map_path,"/path");


   //search
   apex_utils::get_param(node,"trajectory_generator.search_height_min",config.search.search_height_min,0.0);
   apex_utils::get_param(node,"trajectory_generator.search_height_max",config.search.search_height_max,0.5);
   apex_utils::get_param(node,"trajectory_generator.search_radius",config.search.search_radius,0.2);
   apex_utils::get_param(node,"trajectory_generator.robot_radius",config.search.robot_radius,0.3);
   apex_utils::get_param(node,"trajectory_generator.robot_radius_dash",config.search.robot_radius_dash,0.3);

   RCLCPP_INFO(kLogger, "[Manager Init] map/occ_file_path: %s", config.map.occ_map_path.c_str());
   RCLCPP_INFO(kLogger, "[Manager Init] map/bev_file_path: %s", config.map.bev_map_path.c_str());
   RCLCPP_INFO(kLogger, "[Manager Init] map/distance_map_file_path: %s", config.map.distance_map_path.c_str());

   m_eng = std::default_random_engine(m_rd());
   m_rand_pos = std::uniform_real_distribution<double>(-1.0, 1.0);

   sentryColor = teamColor::red;

   global_map.reset(new GlobalMap);
   global_map->initGridMap(node, config);

   astar_path_finder.reset(new AstarPathFinder);
   astar_path_finder->initGridMap(node, global_map);

   path_smoother.reset(new Smoother);
   path_smoother->setGlobalMap(global_map);

   reference_path.reset(new Refenecesmooth);
   reference_path->init(global_map);

   topo_prm.reset(new TopoSearcher);
   topo_prm->init(global_map);

   global_map->setRadiusDash(config.search.robot_radius_dash);  // 动态障碍物膨胀半径设置
}