#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// Structure to hold each trajectory point.
struct TrajectoryPoint {
  double timestamp;
  double x;
  double y;
  double z;
};

// Function to read trajectory points from a CSV file.
// Assumes each line is: timestamp,x,y,z
std::vector<TrajectoryPoint> readTrajectoryFile(const std::string& filename) {
  std::vector<TrajectoryPoint> points;
  std::ifstream infile(filename.c_str());
  if (!infile.is_open()) {
    ROS_ERROR("Could not open file: %s", filename.c_str());
    return points;
  }
  
  std::string line;
  // Skip the header line
  std::getline(infile, line);
  
  // Read the rest of the file
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    TrajectoryPoint pt;
    std::string token;
    if (std::getline(ss, token, ',')) pt.timestamp = std::stod(token);
    if (std::getline(ss, token, ',')) pt.x = std::stod(token);
    if (std::getline(ss, token, ',')) pt.y = std::stod(token);
    if (std::getline(ss, token, ',')) pt.z = std::stod(token);
    points.push_back(pt);
  }
  return points;
}

// Function to convert trajectory points to marker array
visualization_msgs::MarkerArray trajectoryToMarkerArray(const std::vector<TrajectoryPoint>& trajectory) {
    visualization_msgs::MarkerArray marker_array;
    
    // Add a deletion marker to clear previous markers
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "odom";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "trajectory";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // Process each trajectory point
    int id = 0;
    for (const auto& pt : trajectory) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Directly assign trajectory point to marker position
        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        marker.pose.position.z = pt.z;
        
        // Set marker orientation, scale, and color.
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_reader_publisher");
  ros::NodeHandle nh;

  // Publisher for the MarkerArray.
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("robot_saved_path", 1);

  // Initialize tf2 listener to perform transformations.
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Retrieve the filename from a parameter or use a default.
  std::string filename;
  nh.param<std::string>("trajectory_file", filename, "trajectory.csv");

  // Publish the marker array repeatedly.
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    // Read trajectory data from file in each iteration
    std::vector<TrajectoryPoint> trajectory = readTrajectoryFile(filename);
    
    // Convert trajectory to marker array and publish
    visualization_msgs::MarkerArray marker_array = trajectoryToMarkerArray(trajectory);
    marker_pub.publish(marker_array);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
