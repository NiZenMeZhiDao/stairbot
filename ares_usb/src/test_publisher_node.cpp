#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <string>
#include <vector>
#include <cmath> // For std::sin

using namespace std::chrono_literals;

/**
 * @class TestActionPublisher
 * @brief 一个简单节点，用于持续发布测试用的 JointState 消息到 /action 话题。
 */
class TestActionPublisher : public rclcpp::Node
{
public:
    TestActionPublisher() : Node("test_action_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/action", 10);

        // 创建一个周期为 100ms (10Hz) 的定时器来持续发布消息
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TestActionPublisher::publish_message, this));

        RCLCPP_INFO(this->get_logger(), "Test publisher started. Publishing messages continuously at 10Hz...");
    }

private:
    /**
     * @brief 构建并发布动态的 JointState 消息。
     */
    void publish_message()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {
            "FL_thigh_joint_i", "FL_thigh_joint_o", "FR_thigh_joint_i",
            "FR_thigh_joint_o", "waist_joint",      "RL_thigh_joint_i",
            "RL_thigh_joint_o", "RR_thigh_joint_i", "RR_thigh_joint_o"};
        
        // 发送一组固定的目标角度值
        message.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};

        // RCLCPP_INFO(this->get_logger(), "Publishing test joint state to /action topic...");
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestActionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 