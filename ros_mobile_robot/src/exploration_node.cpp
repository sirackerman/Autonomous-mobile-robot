//
// Created by ariano on 24-11-6.
//
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <vector>
#include <queue>
#include <cmath>

class ExplorationNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher goal_pub_;
    tf::TransformListener tf_listener_;

    nav_msgs::OccupancyGrid current_map_;
    geometry_msgs::Pose current_pose_;

    // Parameters
    double min_frontier_size_;
    double robot_radius_;
    double information_gain_threshold_;

    struct Frontier {
        int x, y;
        double information_gain;

        Frontier(int x_, int y_, double gain) : x(x_), y(y_), information_gain(gain) {}

        bool operator<(const Frontier& other) const {
            return information_gain < other.information_gain;
        }
    };

public:
    ExplorationNode() : nh_("~") {
        // Get parameters
        nh_.param("min_frontier_size", min_frontier_size_, 10.0);
        nh_.param("robot_radius", robot_radius_, 0.3);
        nh_.param("information_gain_threshold", information_gain_threshold_, 0.3);

        // Setup subscribers and publishers
        map_sub_ = nh_.subscribe("/map", 1, &ExplorationNode::mapCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    }

    bool isValidCell(int x, int y) {
        return x >= 0 && x < current_map_.info.width &&
               y >= 0 && y < current_map_.info.height;
    }

    double calculateInformationGain(int x, int y) {
        double gain = 0;
        int radius = 5;  // Check surrounding cells

        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                if (!isValidCell(x + dx, y + dy)) continue;

                int idx = (y + dy) * current_map_.info.width + (x + dx);
                if (current_map_.data[idx] == -1) {  // Unknown cell
                    gain += 1.0;
                }
            }
        }
        return gain;
    }

    std::vector<Frontier> findFrontiers() {
        std::vector<Frontier> frontiers;
        std::vector<bool> visited(current_map_.data.size(), false);

        // Find frontier cells (free cells next to unknown space)
        for (int y = 1; y < current_map_.info.height - 1; y++) {
            for (int x = 1; x < current_map_.info.width - 1; x++) {
                int idx = y * current_map_.info.width + x;

                if (visited[idx] || current_map_.data[idx] != 0) continue;

                // Check if cell is at frontier (adjacent to unknown space)
                bool is_frontier = false;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int neighbor_idx = (y + dy) * current_map_.info.width + (x + dx);
                        if (current_map_.data[neighbor_idx] == -1) {
                            is_frontier = true;
                            break;
                        }
                    }
                    if (is_frontier) break;
                }

                if (is_frontier) {
                    double gain = calculateInformationGain(x, y);
                    if (gain > information_gain_threshold_) {
                        frontiers.emplace_back(x, y, gain);
                    }
                }

                visited[idx] = true;
            }
        }

        return frontiers;
    }

    void sendExplorationGoal(const Frontier& frontier) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        // Convert grid coordinates to world coordinates
        goal.pose.position.x = frontier.x * current_map_.info.resolution +
                             current_map_.info.origin.position.x;
        goal.pose.position.y = frontier.y * current_map_.info.resolution +
                             current_map_.info.origin.position.y;
        goal.pose.position.z = 0;

        // Set orientation to face the unknown space
        goal.pose.orientation.w = 1.0;

        goal_pub_.publish(goal);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        current_map_ = *map;

        // Get current robot pose
        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("map", "base_link",
                                       ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return;
        }

        // Find frontiers
        std::vector<Frontier> frontiers = findFrontiers();

        if (!frontiers.empty()) {
            // Sort frontiers by information gain
            std::sort(frontiers.begin(), frontiers.end());

            // Select the frontier with highest information gain
            sendExplorationGoal(frontiers.back());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_node");
    ExplorationNode explorer;
    ros::spin();
    return 0;
}