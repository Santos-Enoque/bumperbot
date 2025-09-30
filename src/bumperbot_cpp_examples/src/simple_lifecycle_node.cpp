#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class SimpleLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SimpleLifeCycleNode(const std::string &node_name, bool intra_process_comms = false) : LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            std::bind(&SimpleLifeCycleNode::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscription created, waiting for lifecycle transition to 'active' to start receiving messages.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        subscription_.reset();
        RCLCPP_INFO(this->get_logger(), "Subscription destroyed.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        subscription_.reset();
        RCLCPP_INFO(this->get_logger(), "Cleanup complete, subscription reset.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(this->get_logger(), "Deactivation complete, subscription reset.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);
        RCLCPP_INFO(this->get_logger(), "Node activated, now receiving messages.");
        std::this_thread::sleep_for(1s); // Simulate some activation delay
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        auto state = this->get_current_state();
        if (state.label() == "active")
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received message while not active, ignoring.");
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    std::shared_ptr<SimpleLifeCycleNode> node = std::make_shared<SimpleLifeCycleNode>("simple_lifecycle_node");
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}