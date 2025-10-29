#include "human_tracker_ros2/clustering/group_cluster.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace human_tracker_ros2
{

GroupCluster::GroupCluster()
  : next_group_id_(1)
{
  // Use default parameters
}

GroupCluster::GroupCluster(const GroupClusterParams& params)
  : params_(params), next_group_id_(1)
{
}

void GroupCluster::setParameters(const GroupClusterParams& params)
{
  params_ = params;
}

GroupClusterParams GroupCluster::getParameters() const
{
  return params_;
}

msg::GroupStates GroupCluster::clusterHumans(const msg::TrackedHumans& tracked_humans, double timestamp)
{
  msg::GroupStates group_states;
  group_states.header.stamp = tracked_humans.header.stamp;
  group_states.header.frame_id = tracked_humans.header.frame_id;
  group_states.total_groups = 0;
  
  // If no humans, return empty groups
  if (tracked_humans.humans.empty()) {
    return group_states;
  }
  
  // Extract positions for clustering
  std::vector<geometry_msgs::msg::Point> positions;
  for (const auto& human : tracked_humans.humans) {
    positions.push_back(human.pose.position);
  }
  
  // Perform clustering
  std::vector<std::vector<int>> clusters = performClustering(positions);
  
  // Create group states from clusters
  std::vector<msg::GroupState> all_groups;
  int group_id = next_group_id_;
  
  // Create all group states first (no limit yet)
  for (const auto& cluster : clusters) {
    if (cluster.empty()) continue;
    
    msg::GroupState group_state = createGroupState(cluster, tracked_humans, group_id, timestamp);
    all_groups.push_back(group_state);
    group_id++;
  }
  
  // Sort groups by distance from reference point to get closest groups
  std::sort(all_groups.begin(), all_groups.end(), 
    [this](const msg::GroupState& a, const msg::GroupState& b) {
      double dx_a = a.position.x - params_.reference_x;
      double dy_a = a.position.y - params_.reference_y;
      double dist_a = std::sqrt(dx_a * dx_a + dy_a * dy_a);
      
      double dx_b = b.position.x - params_.reference_x;
      double dy_b = b.position.y - params_.reference_y;
      double dist_b = std::sqrt(dx_b * dx_b + dy_b * dy_b);
      
      return dist_a < dist_b;  // Closest first
    });
  
  // Take only the closest max_groups (15) groups
  size_t max_groups_to_take = std::min(all_groups.size(), static_cast<size_t>(params_.max_groups));
  for (size_t i = 0; i < max_groups_to_take; ++i) {
    group_states.groups.push_back(all_groups[i]);
  }
  
  group_states.total_groups = static_cast<int>(group_states.groups.size());
  next_group_id_ = group_id;
  
  return group_states;
}

std::vector<std::vector<int>> GroupCluster::performClustering(
  const std::vector<geometry_msgs::msg::Point>& positions)
{
  std::vector<std::vector<int>> clusters;
  std::vector<bool> visited(positions.size(), false);
  
  for (size_t i = 0; i < positions.size(); ++i) {
    if (!visited[i]) {
      std::vector<int> cluster;
      expandCluster(static_cast<int>(i), cluster, visited, positions);
      
      if (!cluster.empty()) {
        clusters.push_back(cluster);
      }
    }
  }
  
  // If treat_individual_as_group is false and we have single-human clusters,
  // we still need to include them as per requirements (individual = group)
  return clusters;
}

void GroupCluster::expandCluster(int human_idx, 
                                std::vector<int>& cluster,
                                std::vector<bool>& visited,
                                const std::vector<geometry_msgs::msg::Point>& positions)
{
  if (visited[human_idx]) return;
  
  visited[human_idx] = true;
  cluster.push_back(human_idx);
  
  // Find all nearby humans
  for (size_t i = 0; i < positions.size(); ++i) {
    if (visited[i]) continue;
    
    double distance = calculateDistance(positions[human_idx], positions[i]);
    if (distance <= params_.max_group_distance) {
      expandCluster(static_cast<int>(i), cluster, visited, positions);
    }
  }
}

msg::GroupState GroupCluster::createGroupState(
  const std::vector<int>& member_indices,
  const msg::TrackedHumans& tracked_humans,
  int group_id,
  double timestamp)
{
  (void)timestamp; // Suppress unused parameter warning
  
  msg::GroupState group_state;
  group_state.header.stamp = tracked_humans.header.stamp;
  group_state.header.frame_id = tracked_humans.header.frame_id;
  group_state.group_id = group_id;
  group_state.member_count = static_cast<int>(member_indices.size());
  
  // Collect member data
  std::vector<geometry_msgs::msg::Point> positions;
  std::vector<geometry_msgs::msg::Vector3> velocities;
  std::vector<int> track_ids;
  
  for (int idx : member_indices) {
    const auto& human = tracked_humans.humans[idx];
    positions.push_back(human.pose.position);
    velocities.push_back(human.velocity);
    track_ids.push_back(human.track_id);
  }
  
  // Calculate group centroid
  group_state.position = calculateCentroid(positions);
  
  // Calculate group velocity
  group_state.velocity = calculateGroupVelocity(velocities);
  
  // Calculate safety radius
  group_state.safety_radius = calculateSafetyRadius(
    positions, group_state.position, params_.safety_radius_factor,
    params_.min_safety_radius, params_.max_safety_radius);
  
  // Store member IDs
  group_state.member_ids = track_ids;
  
  return group_state;
}

double GroupCluster::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                     const geometry_msgs::msg::Point& p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

geometry_msgs::msg::Point GroupCluster::calculateCentroid(
  const std::vector<geometry_msgs::msg::Point>& points)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.0;
  centroid.y = 0.0;
  centroid.z = 0.0;
  
  if (points.empty()) return centroid;
  
  for (const auto& point : points) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
  }
  
  double count = static_cast<double>(points.size());
  centroid.x /= count;
  centroid.y /= count;
  centroid.z /= count;
  
  return centroid;
}

geometry_msgs::msg::Vector3 GroupCluster::calculateGroupVelocity(
  const std::vector<geometry_msgs::msg::Vector3>& velocities)
{
  geometry_msgs::msg::Vector3 group_velocity;
  group_velocity.x = 0.0;
  group_velocity.y = 0.0;
  group_velocity.z = 0.0;
  
  if (velocities.empty()) return group_velocity;
  
  for (const auto& velocity : velocities) {
    group_velocity.x += velocity.x;
    group_velocity.y += velocity.y;
    group_velocity.z += velocity.z;
  }
  
  double count = static_cast<double>(velocities.size());
  group_velocity.x /= count;
  group_velocity.y /= count;
  group_velocity.z /= count;
  
  return group_velocity;
}

double GroupCluster::calculateSafetyRadius(
  const std::vector<geometry_msgs::msg::Point>& positions, 
  const geometry_msgs::msg::Point& centroid,
  double safety_factor, double min_radius, double max_radius)
{
  if (positions.empty()) return min_radius;
  
  // For single human, use minimum safety radius
  if (positions.size() == 1) {
    return min_radius;
  }
  
  // Calculate maximum distance from centroid to any member
  double max_distance = 0.0;
  for (const auto& pos : positions) {
    double distance = calculateDistance(pos, centroid);
    max_distance = std::max(max_distance, distance);
  }
  
  // Apply safety factor and clamp to bounds
  double safety_radius = max_distance * safety_factor;
  safety_radius = std::max(safety_radius, min_radius);
  safety_radius = std::min(safety_radius, max_radius);
  
  return safety_radius;
}

} // namespace human_tracker_ros2