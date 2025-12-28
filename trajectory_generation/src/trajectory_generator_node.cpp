#include "trajectory_generation/GridMap.hpp"
#include "trajectory_generation/plannerManager.hpp"

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator") {}

    void init() {
        planner_manager::init(shared_from_this());
        RCLCPP_INFO(get_logger(), "Planner config loaded successfully");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGeneratorNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
}