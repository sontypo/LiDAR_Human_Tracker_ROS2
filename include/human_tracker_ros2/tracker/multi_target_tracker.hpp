#ifndef HUMAN_TRACKER_ROS2_MULTI_TARGET_TRACKER_HPP
#define HUMAN_TRACKER_ROS2_MULTI_TARGET_TRACKER_HPP

#include "kalman_filter.hpp"
#include "human_tracker_ros2/msg/detected_human.hpp"
#include "human_tracker_ros2/msg/tracked_human.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>

namespace human_tracker_ros2
{

class MultiTargetTracker
{
public:
  using DetectionArray = std::vector<human_tracker_ros2::msg::DetectedHuman>;
  using TrackedHumanArray = std::vector<human_tracker_ros2::msg::TrackedHuman>;
  
  MultiTargetTracker();
  
  // Process new detections and update tracks
  TrackedHumanArray update(const DetectionArray& detections, double timestamp);
  
  // Set tracker parameters
  void setParameters(const std::vector<rclcpp::Parameter> & parameters);

private:
  // Track class to associate detections with Kalman filters
  struct Track {
    uint32_t id;
    KalmanFilter kf;
    uint32_t age;
    uint32_t hits;
    uint32_t misses;
    float confidence;
    uint8_t status;  // 0=new, 1=tracked, 2=occluded, 3=lost
    std::string frame_id;
  };
  
  // Associate detections to existing tracks
  void associateDetectionsToTracks(
    const DetectionArray& detections,
    std::map<int, int>& assignment,
    std::vector<int>& unassigned_tracks,
    std::vector<int>& unassigned_detections);
  
  // Create new tracks from unassigned detections
  void createNewTracks(
    const DetectionArray& detections,
    const std::vector<int>& unassigned_detections,
    double timestamp);
  
  // Update track states
  void updateTracks(
    const DetectionArray& detections,
    const std::map<int, int>& assignment,
    const std::vector<int>& unassigned_tracks,
    double timestamp);
  
  // Remove old tracks
  void removeOldTracks();
  
  // Convert tracks to ROS messages
  TrackedHumanArray tracksToMsg();
  
  // Current active tracks
  std::vector<Track> tracks_;
  
  // Tracker parameters
  float max_distance_;  // Maximum association distance
  uint32_t max_age_;    // Maximum track age before removal
  uint32_t min_hits_;   // Minimum hits to confirm a track
  uint32_t next_id_;    // ID counter for new tracks
  
  // Enhanced Kalman filter parameters for new tracks
  std::map<std::string, double> kalman_parameters_;
};

} // namespace human_tracker_ros2

#endif // HUMAN_TRACKER_ROS2_MULTI_TARGET_TRACKER_HPP