#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>

class SimpleQOSPublisher : public rclcpp::Node
{
public:
    SimpleQOSPublisher() : Node("simple_qos_publisher"), count_(0), qos_profile_pub_(10)
    {
        declare_parameter<std::string>("reliability", "system_default");
        declare_parameter<std::string>("durability", "system_default");

        const auto reliability = get_parameter("reliability").as_string();
        const auto durability = get_parameter("durability").as_string();

        if (reliability == "best_effort") {
            qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            RCLCPP_INFO(get_logger(), "Publisher reliability set to BEST_EFFORT");
        } else if (reliability == "reliable") {
            qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            RCLCPP_INFO(get_logger(), "Publisher reliability set to RELIABLE");
        } else if (reliability == "system_default") {
            qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(), "Publisher reliability set to SYSTEM_DEFAULT");
        } else {
            RCLCPP_WARN(get_logger(), "Unknown reliability setting '%s', using SYSTEM_DEFAULT", reliability.c_str()); qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            return;
        }


        if(durability == "transient_local") {
            qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            RCLCPP_INFO(get_logger(), "Publisher durability set to TRANSIENT_LOCAL");
        } else if (durability == "volatile") {
            qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            RCLCPP_INFO(get_logger(), "Publisher durability set to VOLATILE");
        } else if (durability == "system_default") {
            qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(), "Publisher durability set to SYSTEM_DEFAULT");
        } else {
            RCLCPP_WARN(get_logger(), "Unknown durability setting '%s', using SYSTEM_DEFAULT", durability.c_str()); qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
            return;
        }

        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", qos_profile_pub_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SimpleQOSPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    unsigned int count_ = 0;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_profile_pub_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleQOSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}