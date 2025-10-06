#include <algorithm>
#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{
    PdMotionPlanner::PdMotionPlanner() : Node("pd_motion_planner_node"),
                                         kp_(2.0), kd_(1.0), step_size_(0.2), max_linear_vel_(0.3), max_angular_vel_(1.0), prev_angular_error_(0.0), prev_linear_error_(0.0)
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
            "/a_star/path", 10, std::bind(&PdMotionPlanner::pathCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        next_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/pd/next_pose", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PdMotionPlanner::controlLoop, this));
        
            last_cycle_time_ = get_clock()->now(); 
    }

    void PdMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        global_plan_ = *path;
    }

    void PdMotionPlanner::controlLoop()
    {
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path received yet.");
            return;
        }

        geometry_msgs::msg::TransformStamped robot_pose;
        try
        {
            robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_footprint': %s", ex.what());
            return;
        }

        if(transformPlan(robot_pose.header.frame_id)) {
            RCLCPP_INFO(this->get_logger(), "Transformed plan to frame %s", robot_pose.header.frame_id.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to transform plan to frame %s", robot_pose.header.frame_id.c_str());
            return;
        }

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header = robot_pose.header;
        current_pose.pose.position.x = robot_pose.transform.translation.x;
        current_pose.pose.position.y = robot_pose.transform.translation.y;
        current_pose.pose.position.z = robot_pose.transform.translation.z;
        current_pose.pose.orientation = robot_pose.transform.rotation;

        geometry_msgs::msg::PoseStamped next_pose = getNextPose(current_pose);
        double dx = next_pose.pose.position.x - current_pose.pose.position.x;
        double dy = next_pose.pose.position.y - current_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if(distance <= 0.1) {
            RCLCPP_INFO(this->get_logger(), "Reached the goal.");
            global_plan_.poses.clear();
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_publisher_->publish(stop_msg);
            return;
        }

        next_pose_publisher_->publish(next_pose);
        tf2::Transform tf_current, tf_next, tf_error;
        tf2::fromMsg(current_pose.pose, tf_current);
        tf2::fromMsg(next_pose.pose, tf_next);

        tf_error = tf_current.inverse() * tf_next;
        
        double linear_error = tf_error.getOrigin().getX();
        double angular_error = tf_error.getOrigin().getY();

        double dt = (get_clock()->now() - last_cycle_time_).seconds();
        double linear_error_derivative = (linear_error - prev_linear_error_) / dt;
        double angular_error_derivative = (angular_error - prev_angular_error_) / dt;

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative, -max_linear_vel_, max_linear_vel_);
        cmd_vel.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative, -max_angular_vel_, max_angular_vel_);
        cmd_vel_publisher_->publish(cmd_vel);

        last_cycle_time_ = get_clock()->now();
        prev_linear_error_ = linear_error;
        prev_angular_error_ = angular_error;
    }

    bool PdMotionPlanner::transformPlan(const std::string &frame)
    {
        if (global_plan_.header.frame_id == frame)
        {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform plan from %s to %s: %s",
                        global_plan_.header.frame_id.c_str(), frame.c_str(), ex.what());
            return false;
        }

        for(auto & pose_stamped : global_plan_.poses)
        {
            tf2::doTransform(pose_stamped, pose_stamped, transform);
        }

        global_plan_.header.frame_id = frame;
        return true; 
    }

    geometry_msgs::msg::PoseStamped PdMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & current_pose) {
        auto next_pose = global_plan_.poses.back();

        for(auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) {
            double dx = pose_it->pose.position.x - current_pose.pose.position.x;
            double dy = pose_it->pose.position.y - current_pose.pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            if(distance > step_size_) {
                next_pose = *pose_it;
            } else {
                break;
            }
        }

        return next_pose;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PdMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}