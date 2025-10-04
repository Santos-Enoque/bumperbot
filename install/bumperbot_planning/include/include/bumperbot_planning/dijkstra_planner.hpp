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
     struct GraphNode { 
        int x;
        int y;
        int cost;
        std::shared_ptr<GraphNode> prev;


        GraphNode(int x, int y )
            : x(x), y(y), cost(0) {}

        GraphNode() : GraphNode(0, 0) {}

        bool operator>(const GraphNode &other) const {
            return cost > other.cost;
        }

        bool operator==(const GraphNode &other) const {
            return x == other.x && y == other.y;
        }

        GraphNode operator+(std::pair<int, int> other) const {
            return GraphNode(x + other.first, y + other.second);
        }


     };

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
        GraphNode worldToGrid(const geometry_msgs::msg::Pose &pose);
        geometry_msgs::msg::Pose gridToWorld(const GraphNode &node);
        bool poseOnMap(const GraphNode &node);
        unsigned int poseToCell(const GraphNode &node);

        nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal);
    };
}
#endif // BUMPERBOT_PLANNING__DIJKSTRA_PLANNER_HPP_