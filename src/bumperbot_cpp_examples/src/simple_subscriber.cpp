#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class SimpleSubscriber : public rclcpp::Node
{
public:
    SimpleSubscriber() : Node("simple_subscriber"), qos_profile_sub_(10)
    {
        declare_parameter<std::string>("reliability", "system_default");
        declare_parameter<std::string>("durability", "system_default");

        const auto reliability = get_parameter("reliability").as_string();
        const auto durability = get_parameter("durability").as_string();

        if (reliability == "best_effort") {
            qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            RCLCPP_INFO(get_logger(), "Subscriber reliability set to BEST_EFFORT");
        } else if (reliability == "reliable") {
            qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            RCLCPP_INFO(get_logger(), "Subscriber reliability set to RELIABLE");
        } else if (reliability == "system_default") {
            qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(), "Subscriber reliability set to SYSTEM_DEFAULT");
        } else {
            RCLCPP_WARN(get_logger(), "Unknown reliability setting '%s', using SYSTEM_DEFAULT", reliability.c_str()); qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            return;
        }


        if(durability == "transient_local") {
            qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            RCLCPP_INFO(get_logger(), "Subscriber durability set to TRANSIENT_LOCAL");
        } else if (durability == "volatile") {
            qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            RCLCPP_INFO(get_logger(), "Subscriber durability set to VOLATILE");
        } else if (durability == "system_default") {
            qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(), "Subscriber durability set to SYSTEM_DEFAULT");
        } else {
            RCLCPP_WARN(get_logger(), "Unknown durability setting '%s', using SYSTEM_DEFAULT", durability.c_str()); qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
            return;
        }
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    rclcpp::QoS qos_profile_sub_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}