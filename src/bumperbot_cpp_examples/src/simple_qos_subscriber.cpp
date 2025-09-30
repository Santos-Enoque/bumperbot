#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class SimpleQOSSubscriber : public rclcpp::Node
{
public:
    SimpleQOSSubscriber() : Node("simple_qos_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            std::bind(&SimpleQOSSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleQOSSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}