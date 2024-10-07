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
    /**
     * @brief Constructor for the CylinderDetector node.
     * 
     * Initializes the CylinderDetector node by subscribing to the laser scan topic
     * and setting up a publisher for visualization markers. The default cylinder 
     * diameter is set to 0.3 meters.
     */

    CylinderDetector();

private:
    /**
     * @brief Subscribe to the laser scan topic.
     * 
     * The callback function `scan_callback` is called whenever a new 
     * LaserScan message is received.
     * 
     * @param /scan The topic to subscribe to.
     * @param 10 The queue size for the subscription.
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    /**
     * @brief Detects a cylindrical object in the laser scan data.
     * 
     * This function processes the laser scan data to find clusters of points 
     * that may represent a cylindrical object. It groups points that are close
     * together and checks if they form a circular cluster.
     * 
     * @param msg Shared pointer to the received LaserScan message.
     * @return true If a cylinder is detected.
     * @return false If no cylinder is detected.
     */
    bool detect_cylinder(const sensor_msgs::msg::LaserScan::SharedPtr &msg);
    /**
     * @brief Checks if two laser scan ranges are close enough to be considered part of the same cluster.
     * 
     * This function compares two range values from the laser scan to determine if they
     * are within a given threshold of each other.
     * 
     * @param r1 The first range value.
     * @param r2 The second range value.
     * @param threshold The maximum allowed difference between the two range values.
     * @return true If the two ranges are close enough.
     * @return false If the two ranges are too far apart.
     */
    bool is_close(double r1, double r2, double threshold = 0.2);
    /**
     * @brief Checks if a given cluster of laser scan points forms a circular shape.
     * 
     * This function estimates the diameter of a cluster of points by using the law of cosines
     * and compares it to the expected cylinder diameter. It also checks if the midpoint of the 
     * cluster is closer than the edge points, indicating a cylindrical shape.
     * 
     * @param cluster A vector of range values representing the cluster.
     * @param angle_increment The angular increment between successive laser scan points.
     * @return true If the cluster forms a circular shape.
     * @return false If the cluster does not form a circular shape.
     */
    bool is_circular_cluster(const std::vector<double> &cluster, double angle_increment);
    /**
     * @brief Publishes a visualization marker to represent the detected cylinder.
     * 
     * This function creates and configures a cylinder marker for RViz visualization
     * and publishes it to the /cylinder_marker topic. The marker's position is set 
     * based on the detected point and its scale is adjusted to match the cylinder's diameter.
     * 
     * @param point The 2D coordinates of the cylinder in the map frame.
     * @param diameter The diameter of the detected cylinder.
     */
    void publish_marker(geometry_msgs::msg::Point32 point, double diameter);
   /**
     * @brief Transforms a point from the laser scan frame to the map frame.
     * 
     * This function takes a point from the laser scan data and transforms it into 
     * the map frame using the tf2 transform system. It looks up the transform between
     * the laser frame and the map frame, then applies the transformation to the point.
     * 
     * @param msg Shared pointer to the received LaserScan message.
     * @param i The index of the point in the laser scan data.
     * @return geometry_msgs::msg::Point32 The transformed point in the map frame.
     */
    geometry_msgs::msg::Point32 transformScanToMapFrame(const sensor_msgs::msg::LaserScan::SharedPtr &msg, int i);
    /**
     * @brief Fits a circle to a segment of laser scan data.
     * 
     * This function attempts to fit a circle to a set of points derived from laser scan data 
     * between the given start and end indices. It uses a least-squares method to compute the 
     * circle's center and radius. The points are transformed into the map frame before fitting.
     * 
     * @param msg Shared pointer to the received LaserScan message.
     * @param start The index of the first point in the laser scan data to use.
     * @param end The index of the last point in the laser scan data to use.
     * @return Circle The fitted circle containing the center (a, b), radius (r), 
     *         and the number of iterations (j) performed during the fitting.
     */
    Circle checkCircleFit(const sensor_msgs::msg::LaserScan::SharedPtr &msg, int start, int end);


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    double cylinder_diameter_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif // CYLINDER_DETECTOR_HPP
