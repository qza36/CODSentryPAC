#include "trajectory_generation/planner_config.hpp"
#include "getparm_utils.hpp"
#include "trajectory_generation/plannerManger.hpp"

namespace planner_manger {
    std::shared_ptr<GlobalMap> global_map;
}

void planner_manger::init(rclcpp::Node::SharedPtr node)
{
   PlannerConfig config;
   //地图参数
   apex_utils::get_param(node,"trajectory_generator.height_bias",config.map.height_bias,-0.5);
   apex_utils::get_param(node,"trajectory_generator.height_interval",config.map.height_interval,2.0);
   apex_utils::get_param(node,"trajectory_generator.heiht_threshold",config.map.height_threshold,0.1);
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
   global_map = std::make_shared<GlobalMap>();
   global_map->initGridMap(node,config);
}