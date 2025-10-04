#ifndef BUMPERBOT_PLANNING__DIJKSTRA_PLANNER_HPP_
#define BUMPERBOT_PLANNING__DIJKSTRA_PLANNER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_planning
{
    class DijkstraPlanner : public rclcpp::Node
    {
    public:
        DijkstraPlanner();

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        nav_msgs::msg::OccupancyGrid::SharedPtr visited_map_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

        nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal);
    };
}
#endif // BUMPERBOT_PLANNING__DIJKSTRA_PLANNER_HPP_