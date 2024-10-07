#include "../include/object_identifier_node/object_identifier.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp> 
#include <vector>
#include <cmath>

#include <geometry_msgs/msg/point32.hpp>  
#include <tf2_ros/transform_listener.h>    
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>     
#include <tf2_ros/buffer.h>


CylinderDetector::CylinderDetector() 
    : Node("cylinder_detector"),
        cylinder_diameter_(0.3), //meters
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
{
    // Subscribe to the laser scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&CylinderDetector::scan_callback, this, std::placeholders::_1));
    
    // Publisher for visualization markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);
}

    

void CylinderDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (detect_cylinder(msg)) {
        RCLCPP_INFO(this->get_logger(), "Cylinder detected!");
    } 
    //else {
    //     RCLCPP_INFO(this->get_logger(), "No cylinder detected.");
    // }
}


bool CylinderDetector::detect_cylinder(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
{
    double angle_increment = msg->angle_increment;
    const int min_cluster_points = 7;  // Minimum number of points in a cluster

    // Iterate through ranges to find clusters
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        std::vector<double> cluster;
        double start_range = msg->ranges[i];

        // Collect points into a cluster if they're close to each other
        while (i < msg->ranges.size() && is_close(start_range, msg->ranges[i])) {
            cluster.push_back(msg->ranges[i]);
            start_range = msg->ranges[i]; 
            i++;
        }

        // Check if the cluster forms a circular object (potential cylinder)
        if (cluster.size() >= min_cluster_points) {
            // Circle circle = checkCircleFit(msg, i-cluster.size(), i);
            // geometry_msgs::msg::Point32 transformed_point;
            // transformed_point.x = circle.b;
            // transformed_point.y = circle.a; 
            // transformed_point.z = 0.0;
            // publish_marker(transformed_point, circle.r*2); 

            if (is_circular_cluster(cluster, angle_increment)) {     
                geometry_msgs::msg::Point32 transformed_point = transformScanToMapFrame(msg, i-cluster.size()/2);
                
                publish_marker(transformed_point, cylinder_diameter_); 
                return true;
            }
        }
    }
    return false;
}


bool CylinderDetector::is_close(double r1, double r2, double threshold)
{
    return std::fabs(r1 - r2) < threshold;  // Threshold to group points into a cluster
}


bool CylinderDetector::is_circular_cluster(const std::vector<double> &cluster, double angle_increment)
{
    double angle_span_rads = (cluster.size() * angle_increment); 
    int clusterSize = cluster.size();
    // double angle_span_rads = (cluster.size() * angle_increment) * M_PI/180;
    // double estimated_diameter = angle_span / 2 * range;

    double start = cluster[0];
    double end = cluster[cluster.size()-1];
    double middleRange = cluster[cluster.size()/2];

    double estimated_diameter = sqrt(pow(start, 2) + pow(end, 2) - 2 * start * end * cos(angle_span_rads));
    //could get first range and last range with the angle span there fore we got two sides and an angle 
    // c^2 = a^2 + b^2 - 2abCos C 
    //for cylinder could check middle range and see if its -radius from edges

    // double averageEdges = (start+end)/2;
    // if(middleRange < start && middleRange < end){ 
    //     RCLCPP_INFO(this->get_logger(), "Its not a cylinder!");
    //     return false;
    // }
    
    RCLCPP_INFO(this->get_logger(), "Angle span radians: %f, Estimated Diameter: %f meters, Start range: %f, End Range: %f, Middle Range: %f, Cluster Size: %i", angle_span_rads, estimated_diameter, start, end, middleRange, clusterSize);

    return std::fabs(estimated_diameter - cylinder_diameter_) < 0.05; 
}




void CylinderDetector::publish_marker(geometry_msgs::msg::Point32 point, double diameter)
{
    
    // Create a marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Change to the appropriate frame
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cylinder_detection";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set position and orientation (place in the middle of the detected cluster)
        
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0; // Raise to the height of the cylinder
    marker.pose.orientation.w = 1.0; // No rotation

    marker.scale.x = diameter; // Diameter
    marker.scale.y = diameter; // Diameter
    marker.scale.z = 2; // Height of the cylinder

    marker.color.a = 0.5; // Alpha (transparency)
    marker.color.r = 0.0; // Red
    marker.color.g = 1.0; // Green
    marker.color.b = 0.0; // Blue

    RCLCPP_INFO(this->get_logger(), "Publishing to X: %f Y: %f", point.x, point.y);

    marker_publisher_->publish(marker);
}


geometry_msgs::msg::Point32 CylinderDetector::transformScanToMapFrame(const sensor_msgs::msg::LaserScan::SharedPtr &msg, int i){
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Get the transform between the laser frame and the map frame
        transform = tf_buffer_.lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform warning: %s", ex.what());
    }

    // Create a point for the laser range in the laser scan frame
    geometry_msgs::msg::Point32 point;
    point.x = (msg->ranges[i]+cylinder_diameter_/2) * std::cos(msg->angle_min + msg->angle_increment * i); 
    point.y = (msg->ranges[i]+cylinder_diameter_/2) * std::sin(msg->angle_min + msg->angle_increment * i); //+cylinder_diameter_/2 removed from msg->range[i] in x and y // makes for inactuare conversion but works for the publish marker for circle center case
    point.z = 0.0;

    // Transform the point to the map frame
    geometry_msgs::msg::Point32 transformed_point;
    tf2::doTransform(point, transformed_point, transform);

    return transformed_point;
}


//theres to much noise/not enough points and not enough arc for accurate guesss 
Circle CylinderDetector::checkCircleFit(const sensor_msgs::msg::LaserScan::SharedPtr &msg, int start, int end) {

    Circle circle;
    int IterMAX = 99;
    double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
    double A0, A1, A2, A22, A3, A33;
    double Dy, xnew, x = 0, ynew, y;
    double DET, Xcenter, Ycenter;
//   RCLCPP_INFO(this->get_logger(), 
//             "Cstart tend (a: %i, b: %i)",start,end);


    Data data;
    
    // // Extract points from the LaserScan message
    // for (int i = start; i <= end; i++) {
    //     double range = msg->ranges[i];
    //     if (range < msg->range_min || range > msg->range_max) {
    //         continue; // Skip invalid points
    //     }

    //     // Calculate the X, Y coordinates from the range and angle
    //     double angle = msg->angle_min + i * msg->angle_increment;
    //     double x = range * std::cos(angle);
    //     double y = range * std::sin(angle);

    //     data.X.push_back(x);
    //     data.Y.push_back(y);
    // }
    geometry_msgs::msg::Point32 point;

    for (int i = start; i <= end; i++) {
        point = transformScanToMapFrame(msg,i);
        // publish_marker(point, 0.1);
        data.X.push_back(point.x);
        data.Y.push_back(point.y);
    }
    

    data.n = data.X.size();
    if (data.n < 3) {
        // Not enough points to fit a circle
          RCLCPP_INFO(this->get_logger(), "Not enough valid points to fit a circle.");
         return circle; 
    }

    // Compute sample means
    data.means();

    // Initialize moment sums
    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.0;

    // Compute moments
    for (int i = 0; i < data.n; i++) {
        double Xi = data.X[i] - data.meanX;  // Centered x-coordinates
        double Yi = data.Y[i] - data.meanY;  // Centered y-coordinates
        double Zi = Xi * Xi + Yi * Yi;

        Mxy += Xi * Yi;
        Mxx += Xi * Xi;
        Myy += Yi * Yi;
        Mxz += Xi * Zi;
        Myz += Yi * Zi;
        Mzz += Zi * Zi;
    }
    Mxx /= data.n;
    Myy /= data.n;
    Mxy /= data.n;
    Mxz /= data.n;
    Myz /= data.n;
    Mzz /= data.n;

    // Coefficients of characteristic polynomial
    Mz = Mxx + Myy;
    Cov_xy = Mxx * Myy - Mxy * Mxy;
    Var_z = Mzz - Mz * Mz;
    A3 = 4 * Mz;
    A2 = -3 * Mz * Mz + Mzz;
    A1 = Var_z * Mz + 4 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
    A22 = A2 + A2;
    A33 = A3 + A3 + A3;

    // Newton's method to find the root of the polynomial
    y = A0;
    for (int iter = 0; iter < IterMAX; iter++) {
        Dy = A1 + x * (A22 + A33 * x);
        xnew = x - y / Dy;
        if (std::abs(xnew - x) < std::numeric_limits<double>::epsilon()) break;
        ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
        if (std::abs(ynew) >= std::abs(y)) break;
        x = xnew;
        y = ynew;
    }

    // Compute circle parameters
    DET = x * x - x * Mz + Cov_xy;
    Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / (DET * 2);
    Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / (DET * 2);

    // Set circle properties
    circle.a = Xcenter + data.meanX;
    circle.b = Ycenter + data.meanY;
    circle.r = std::sqrt(Xcenter * Xcenter + Ycenter * Ycenter + Mz);
    circle.s = 0; // You can compute the root mean square error here if needed
    circle.j = IterMAX; // Store the number of iterations
    if (!std::isnan(circle.a) && !std::isnan(circle.b) && !std::isnan(circle.r)) {
        RCLCPP_INFO(this->get_logger(), 
                "Circle found: Center (a: %f, b: %f), Radius: %f, Iterations: %d", 
                circle.a, circle.b, circle.r, circle.j);
        // for(int i = 0; i< data.n; i++){
        //     geometry_msgs::msg::Point32 point;
        //     point.x = data.X[i];
        //     point.y = data.Y[i];

        //     publish_marker(point, 0.1);
        // }
    }

    return circle;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}
