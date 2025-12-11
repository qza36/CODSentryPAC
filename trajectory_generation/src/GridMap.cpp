#include "trajectory_generation/GridMap.hpp"
#include "getparm_utils.hpp"

void GlobalMap::initGridMap(
    rclcpp::Node::SharedPtr node,
    Eigen::Vector3d global_xyz_l,
    Eigen::Vector3d global_xyz_u
    )
{
   //获取地图边界
   gl_xl = global_xyz_l(0);
   gl_yl = global_xyz_l(1);
   gl_zl = global_xyz_l(2);

   gl_xu = global_xyz_u(0);
   gl_yu = global_xyz_u(1);
   gl_zu = global_xyz_u(2);


}
