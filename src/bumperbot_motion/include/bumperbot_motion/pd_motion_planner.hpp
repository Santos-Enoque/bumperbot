#ifndef PD_MOTION_PLANNER_HPP
#define PD_MOTION_PLANNER_HPP
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_motion
{
class PdMotionPlanner : public rclcpp::Node
{
public:
  PdMotionPlanner();

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    double kp_;
    double kd_;
    double step_size_;
    double max_linear_vel_;
    double max_angular_vel_;
    nav_msgs::msg::Path global_plan_;


    void controlLoop();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

};
}  // namespace bumperbot_motion
#endif  // PD_MOTION_PLANNER_HPP