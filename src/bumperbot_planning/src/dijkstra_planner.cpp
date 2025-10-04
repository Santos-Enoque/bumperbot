#include "bumperbot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"

namespace bumperbot_planning
{
    DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_planner")
    {
        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        rclcpp::QoS goal_qos(10);

        rclcpp::QoS path_qos(10);

        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));

        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", goal_qos, std::bind(&DijkstraPlanner::goalCallback, this, std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/dijkstra/path", path_qos);
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visited_map", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }


    void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        map_ = map;
        visited_map_->header.frame_id = map_->header.frame_id;
        visited_map_->info = map_->info;
        visited_map_->data = std::vector<int8_t>(map_->info.width * map_->info.height, -1);
    }

    void DijkstraPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        if (!map_)
        {
            RCLCPP_WARN(this->get_logger(), "Map not received yet.");
            return;
        }

        visited_map_->data = std::vector<int8_t>(map_->info.width * map_->info.height, -1);

        geometry_msgs::msg::TransformStamped map_to_base_tf;

        try {
            map_to_base_tf = tf_buffer_->lookupTransform(
                map_->header.frame_id, "base_footprint", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform 'base_footprint' to 'map': %s", ex.what());
            return;
        }
       
        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        nav_msgs::msg::Path path = plan(map_to_base_pose, pose->pose);

        if(!path.poses.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Path found with %zu poses.", path.poses.size()); 
            path_publisher_->publish(path);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No path found to the goal.");
        }
    }

    nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
    {
        nav_msgs::msg::Path path;
        // Implement Dijkstra's algorithm here to find the shortest path from start to goal
        // using the occupancy grid map_ and update visited_map_ accordingly.
        // For simplicity, this is a placeholder implementation.

        // Placeholder: Return an empty path
        return path;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}