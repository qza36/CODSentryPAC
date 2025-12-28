/**
 * @file trajectory_generator_node.cpp
 * @brief 轨迹生成节点 - 集成状态机和规划器
 */

#include "trajectory_generation/replan_fsm.hpp"

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator") {}

    void init() {
        fsm_.init(shared_from_this());
        RCLCPP_INFO(get_logger(), "Trajectory generator initialized");
    }

private:
    ReplanFSM fsm_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGeneratorNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
