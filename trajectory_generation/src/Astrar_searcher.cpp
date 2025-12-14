#include "trajectory_generation/Astar_searcher.hpp"
#include "trajectory_generation/plannerManger.hpp"

int main(int argc,char** argv)
{

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("test_planner");
    try
    {
        planner_manger::init(node);
    }catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Init Failed: %s", e.what());
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}