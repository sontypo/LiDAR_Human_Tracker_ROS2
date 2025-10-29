#ifndef HUMAN_TRACKER_ROS2_GROUP_CLUSTER_HPP
#define HUMAN_TRACKER_ROS2_GROUP_CLUSTER_HPP

#include <vector>
#include <Eigen/Dense>
#include "human_tracker_ros2/msg/tracked_humans.hpp"
#include "human_tracker_ros2/msg/group_states.hpp"

namespace human_tracker_ros2
{

struct GroupClusterParams
{
  double max_group_distance = 2.0;        // Maximum distance between humans to be in same group (m)
  double min_safety_radius = 0.8;         // Minimum safety radius for individual (m)
  double max_safety_radius = 3.0;         // Maximum safety radius for groups (m)
  double safety_radius_factor = 1.5;      // Factor to calculate safety radius from group spread
  int max_groups = 15;                    // Maximum number of groups to track
  double group_velocity_threshold = 0.1;  // Minimum velocity to consider group moving (m/s)
  bool treat_individual_as_group = true;  // Always treat single human as group
  
  // Reference point for distance calculation (robot position)
  double reference_x = 0.0;               // Reference X position for closest group calculation
  double reference_y = 0.0;               // Reference Y position for closest group calculation
};

class GroupCluster
{
public:
  GroupCluster();
  explicit GroupCluster(const GroupClusterParams& params);
  
  // Main clustering function
  msg::GroupStates clusterHumans(const msg::TrackedHumans& tracked_humans, double timestamp);
  
  // Parameter management
  void setParameters(const GroupClusterParams& params);
  GroupClusterParams getParameters() const;
  
  // Utility functions
  static double calculateDistance(const geometry_msgs::msg::Point& p1, 
                                const geometry_msgs::msg::Point& p2);
  static geometry_msgs::msg::Point calculateCentroid(
    const std::vector<geometry_msgs::msg::Point>& points);
  static geometry_msgs::msg::Vector3 calculateGroupVelocity(
    const std::vector<geometry_msgs::msg::Vector3>& velocities);
  static double calculateSafetyRadius(
    const std::vector<geometry_msgs::msg::Point>& positions, 
    const geometry_msgs::msg::Point& centroid,
    double safety_factor, double min_radius, double max_radius);

private:
  GroupClusterParams params_;
  int next_group_id_;
  
  // Internal clustering methods
  std::vector<std::vector<int>> performClustering(
    const std::vector<geometry_msgs::msg::Point>& positions);
  
  msg::GroupState createGroupState(
    const std::vector<int>& member_indices,
    const msg::TrackedHumans& tracked_humans,
    int group_id,
    double timestamp);
  
  // Distance-based clustering using DBSCAN-like approach
  void expandCluster(int human_idx, 
                    std::vector<int>& cluster,
                    std::vector<bool>& visited,
                    const std::vector<geometry_msgs::msg::Point>& positions);
};

} // namespace human_tracker_ros2

#endif // HUMAN_TRACKER_ROS2_GROUP_CLUSTER_HPP