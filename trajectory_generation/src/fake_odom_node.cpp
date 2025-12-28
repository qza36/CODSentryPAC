/**
 * @file fake_odom_node.cpp
 * @brief 假里程计节点 - 用于RViz测试
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class FakeOdomNode : public rclcpp::Node {
public:
    FakeOdomNode() : Node("fake_odom") {
        declare_parameter("x", 0.0);
        declare_parameter("y", 0.0);

        x_ = get_parameter("x").as_double();
        y_ = get_parameter("y").as_double();

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&FakeOdomNode::publishOdom, this));

        RCLCPP_INFO(get_logger(), "Fake odom at (%.2f, %.2f)", x_, y_);
    }

private:
    void publishOdom() {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = now();
        msg.header.frame_id = "world";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = x_;
        msg.pose.pose.position.y = y_;
        msg.pose.pose.orientation.w = 1.0;
        odom_pub_->publish(msg);
    }

    double x_, y_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdomNode>());
    rclcpp::shutdown();
}
