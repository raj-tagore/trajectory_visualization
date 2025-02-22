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

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_reader_publisher");
  ros::NodeHandle nh;

  // Publisher for the MarkerArray.
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_marker_array", 1);

  // Initialize tf2 listener to perform transformations.
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Retrieve the filename from a parameter or use a default.
  std::string filename;
  nh.param<std::string>("trajectory_file", filename, "trajectory.csv");

  // Read trajectory data from file.
  std::vector<TrajectoryPoint> trajectory = readTrajectoryFile(filename);

  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  // Process each trajectory point.
  for (const auto& pt : trajectory) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";  // Final visualization frame.
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Create a point from the file data.
    geometry_msgs::Point point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    
    // Initially, we assume the data is in a frame called "saved_frame".
    // We transform it to "odom". If your data is already in odom, this step can be skipped.
    geometry_msgs::PointStamped point_in, point_out;
    point_in.header.frame_id = "saved_frame";  
    point_in.header.stamp = ros::Time::now();
    point_in.point = point;

    try {
      // Lookup transformation from "saved_frame" to "odom".
      geometry_msgs::TransformStamped transformStamped =
        tfBuffer.lookupTransform("odom", "saved_frame", ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(point_in, point_out, transformStamped);
      marker.pose.position = point_out.point;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform point: %s", ex.what());
      // Fallback: publish the original point.
      marker.pose.position = point;
    }
    
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

  // Publish the marker array repeatedly.
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    // Update header timestamps if needed.
    for (auto& marker : marker_array.markers) {
      marker.header.stamp = ros::Time::now();
    }
    marker_pub.publish(marker_array);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
