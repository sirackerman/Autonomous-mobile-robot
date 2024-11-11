//
// Created by ariano on 24-11-6.
//
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudToGrid {
private:
    ros::NodeHandle nh_;
    ros::Publisher grid_pub_;
    ros::Subscriber cloud_sub_;

    // Grid parameters
    double resolution_;
    double height_threshold_;
    int grid_width_, grid_height_;
    double grid_origin_x_, grid_origin_y_;

    nav_msgs::OccupancyGrid grid_map_;

public:
    PointCloudToGrid() : nh_("~") {
        // Get parameters
        nh_.param("resolution", resolution_, 0.05);  // 5cm resolution
        nh_.param("height_threshold", height_threshold_, 0.5);  // Points above 0.5m are ignored
        nh_.param("grid_width", grid_width_, 1000);  // 50m width (1000 * 0.05)
        nh_.param("grid_height", grid_height_, 1000);  // 50m height
        nh_.param("grid_origin_x", grid_origin_x_, -25.0);  // Center grid at robot
        nh_.param("grid_origin_y", grid_origin_y_, -25.0);

        // Initialize grid map
        grid_map_.header.frame_id = "map";
        grid_map_.info.resolution = resolution_;
        grid_map_.info.width = grid_width_;
        grid_map_.info.height = grid_height_;
        grid_map_.info.origin.position.x = grid_origin_x_;
        grid_map_.info.origin.position.y = grid_origin_y_;
        grid_map_.info.origin.orientation.w = 1.0;
        grid_map_.data.resize(grid_width_ * grid_height_, -1);

        // Setup publisher and subscriber
        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        cloud_sub_ = nh_.subscribe("/orb_slam3/map_points", 1, &PointCloudToGrid::cloudCallback, this);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        // Reset grid
        std::fill(grid_map_.data.begin(), grid_map_.data.end(), -1);

        // Convert points to grid cells
        for (const auto& point : cloud.points) {
            // Ignore points above height threshold
            if (point.z > height_threshold_) continue;

            // Convert to grid coordinates
            int grid_x = (point.x - grid_map_.info.origin.position.x) / resolution_;
            int grid_y = (point.y - grid_map_.info.origin.position.y) / resolution_;

            // Check if point is within grid bounds
            if (grid_x >= 0 && grid_x < grid_width_ &&
                grid_y >= 0 && grid_y < grid_height_) {
                // Mark cell as occupied
                grid_map_.data[grid_y * grid_width_ + grid_x] = 100;
            }
        }

        // Fill in unknown cells and apply simple dilation for safety
        for (int y = 1; y < grid_height_ - 1; y++) {
            for (int x = 1; x < grid_width_ - 1; x++) {
                int idx = y * grid_width_ + x;
                if (grid_map_.data[idx] == -1) {
                    // Mark unknown cells as free if they're between known cells
                    grid_map_.data[idx] = 0;
                }
                // Simple dilation for occupied cells
                else if (grid_map_.data[idx] == 100) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            int neighbor_idx = (y + dy) * grid_width_ + (x + dx);
                            if (grid_map_.data[neighbor_idx] != 100) {
                                grid_map_.data[neighbor_idx] = 50;  // Mark as potentially occupied
                            }
                        }
                    }
                }
            }
        }

        // Update timestamp and publish
        grid_map_.header.stamp = ros::Time::now();
        grid_pub_.publish(grid_map_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_grid");
    PointCloudToGrid converter;
    ros::spin();
    return 0;
}