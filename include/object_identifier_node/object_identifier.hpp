#ifndef CYLINDER_DETECTOR_HPP
#define CYLINDER_DETECTOR_HPP



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <vector>
#include <cmath>

struct Circle {
    double a, b, r, s; // Center (a, b), radius (r), and root mean square error (s)
    int j; // Number of iterations
};

// Struct to hold the data points (extracted from LaserScan)
struct Data {
    std::vector<double> X, Y;
    int n;
    double meanX, meanY;

    // Compute the mean of the data points
    void means() {
        meanX = 0;
        meanY = 0;
        for (int i = 0; i < n; i++) {
            meanX += X[i];
            meanY += Y[i];
        }
        meanX /= n;
        meanY /= n;
    }
};




class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool detect_cylinder(const sensor_msgs::msg::LaserScan::SharedPtr &msg);
    bool is_close(double r1, double r2, double threshold = 0.2);
    bool is_circular_cluster(const std::vector<double> &cluster, double angle_increment);
    void publish_marker(geometry_msgs::msg::Point32 point, double diameter);
    geometry_msgs::msg::Point32 transformScanToMapFrame(const sensor_msgs::msg::LaserScan::SharedPtr &msg, int i);
    Circle checkCircleFit(const sensor_msgs::msg::LaserScan::SharedPtr &msg, int start, int end);


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    double cylinder_diameter_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif // CYLINDER_DETECTOR_HPP
