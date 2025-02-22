#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "trajectory_visualization/SaveTrajectory.h"  // Custom service header
#include <deque>
#include <fstream>

// Structure to store each trajectory point with its timestamp.
struct TrajectoryPoint {
  ros::Time stamp;
  geometry_msgs::Point position;
};

// Global container for trajectory data.
std::deque<TrajectoryPoint> trajectory_data;
ros::Publisher marker_pub;

// Callback function for receiving odometry messages.
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  TrajectoryPoint pt;
  pt.stamp = msg->header.stamp;
  pt.position = msg->pose.pose.position;
  trajectory_data.push_back(pt);

  // Publish the trajectory as a MarkerArray for visualization in RViz.
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";  // Change to the relevant frame if needed.
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Add each stored position to the marker's point list.
  for (const auto &p : trajectory_data) {
    marker.points.push_back(p.position);
  }
  marker_array.markers.push_back(marker);
  marker_pub.publish(marker_array);
}

// Service callback to save the trajectory data.
// The service request includes the file name and the duration (in seconds).
bool saveTrajectoryCallback(trajectory_visualization::SaveTrajectory::Request &req,
                            trajectory_visualization::SaveTrajectory::Response &res)
{
  ros::Time now = ros::Time::now();
  double duration = req.duration;
  std::deque<TrajectoryPoint> filtered_data;

  // Filter out the trajectory points within the specified duration (going backward from now).
  for (auto it = trajectory_data.rbegin(); it != trajectory_data.rend(); ++it) {
    if ((now - it->stamp).toSec() <= duration) {
      filtered_data.push_front(*it);
    } else {
      break;  // Since the data is in order, we can stop once the condition fails.
    }
  }

  // Save the filtered trajectory to file.
  std::ofstream outfile(req.file_name.c_str());
  if (!outfile.is_open()) {
    res.success = false;
    res.message = "Failed to open file: " + req.file_name;
    return true;
  }

  // Write a header and then each trajectory point as CSV.
  outfile << "timestamp,x,y,z\n";
  for (const auto &pt : filtered_data) {
    outfile << pt.stamp.toSec() << "," << pt.position.x << "," 
            << pt.position.y << "," << pt.position.z << "\n";
  }
  outfile.close();

  res.success = true;
  res.message = "Trajectory saved successfully to " + req.file_name;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_publisher_saver_node");
  ros::NodeHandle nh;

  // Subscribe to the odometry topic (adjust the topic name if necessary).
  ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);

  // Publisher for visualization markers.
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_marker", 1);

  // Advertise the save trajectory service.
  ros::ServiceServer service = nh.advertiseService("save_trajectory", saveTrajectoryCallback);

  ros::spin();
  return 0;
}
