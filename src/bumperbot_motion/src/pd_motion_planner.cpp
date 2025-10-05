#include "bumperbot_motion/pd_motion_planner.hpp"


namespace bumperbot_motion
{
    PdMotionPlanner::PdMotionPlanner() : Node("pd_motion_planner_node"),
    kp_(2.0), kd_(1.0), step_size_(0.2), max_linear_vel_(0.3), max_angular_vel_(1.0)
    {
        declare_parameter<double>("kp", kp_);
        declare_parameter<double>("kd", kd_);
        declare_parameter<double>("step_size", step_size_);
        declare_parameter<double>("max_linear_vel", max_linear_vel_);
        declare_parameter<double>("max_angular_vel", max_angular_vel_);

        kp_ = get_parameter("kp").as_double();
        kd_ = get_parameter("kd").as_double();
        step_size_ = get_parameter("step_size").as_double();
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = get_parameter("max_angular_vel").as_double();

        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_plan", 10, std::bind(&PdMotionPlanner::pathCallback, this, std::placeholders::_1));
    }
}