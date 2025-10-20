#include "human_tracker_ros2/tracker/multi_target_tracker.hpp"
#include <Eigen/Dense>

namespace human_tracker_ros2
{

MultiTargetTracker::MultiTargetTracker()
  : max_distance_(1.0),
    max_age_(10),
    min_hits_(3),
    next_id_(1)
{
  // Initialize tracker
}

void MultiTargetTracker::setParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  std::map<std::string, double> kalman_params;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "max_distance") {
      max_distance_ = param.as_double();
    } else if (param.get_name() == "max_age") {
      max_age_ = param.as_int();
    } else if (param.get_name() == "min_hits") {
      min_hits_ = param.as_int();
    } else if (param.get_name().find("kalman_") == 0) {
      // Handle boolean parameters specially
      if (param.get_name() == "kalman_enable_joseph_form" ||
          param.get_name() == "kalman_enable_outlier_rejection" ||
          param.get_name() == "kalman_enable_velocity_gating") {
        kalman_params[param.get_name()] = param.as_bool() ? 1.0 : 0.0;
      } else {
        // Collect all other Kalman filter parameters as double
        kalman_params[param.get_name()] = param.as_double();
      }
    }
  }
  
  // Apply Kalman filter parameters to all existing tracks
  for (auto& track : tracks_) {
    track.kf.setParameters(kalman_params);
  }
  
  // Store parameters for new tracks
  kalman_parameters_ = kalman_params;
}

MultiTargetTracker::TrackedHumanArray MultiTargetTracker::update(
  const DetectionArray& detections, 
  double timestamp)
{
  // Step 1: Predict step for all existing tracks
  for (auto& track : tracks_) {
    track.kf.predict(timestamp);
  }
  
  // Step 2: Associate detections to tracks
  std::map<int, int> assignment;
  std::vector<int> unassigned_tracks;
  std::vector<int> unassigned_detections;
  associateDetectionsToTracks(detections, assignment, unassigned_tracks, unassigned_detections);
  
  // Step 3: Update tracks with associated detections
  updateTracks(detections, assignment, unassigned_tracks, timestamp);
  
  // Step 4: Create new tracks for unassigned detections
  createNewTracks(detections, unassigned_detections, timestamp);
  
  // Step 5: Remove old tracks
  removeOldTracks();
  
  // Step 6: Convert to ROS messages and return
  return tracksToMsg();
}

void MultiTargetTracker::associateDetectionsToTracks(
  const DetectionArray& detections,
  std::map<int, int>& assignment,
  std::vector<int>& unassigned_tracks,
  std::vector<int>& unassigned_detections)
{
  // Initialize all track indices as unassigned
  for (size_t t = 0; t < tracks_.size(); t++) {
    unassigned_tracks.push_back(t);
  }
  
  // Initialize all detection indices as unassigned
  for (size_t d = 0; d < detections.size(); d++) {
    unassigned_detections.push_back(d);
  }
  
  // Compute cost matrix based on distance
  Eigen::MatrixXd cost_matrix(tracks_.size(), detections.size());
  
  for (size_t t = 0; t < tracks_.size(); t++) {
    const auto& track_pos = tracks_[t].kf.getState().block<3, 1>(0, 0);
    
    for (size_t d = 0; d < detections.size(); d++) {
      const auto& det = detections[d];
      
      // Euclidean distance between track and detection
      Eigen::Vector3d det_pos(det.position.x, det.position.y, det.position.z);
      double distance = (det_pos - track_pos).norm();
      
      cost_matrix(t, d) = distance;
    }
  }
  
  // Apply Hungarian algorithm or greedy assignment
  // For simplicity, we'll use greedy assignment here
  while (!unassigned_tracks.empty() && !unassigned_detections.empty()) {
    // Find minimum distance
    double min_dist = std::numeric_limits<double>::max();
    int min_track = -1;
    int min_detection = -1;
    
    for (size_t i = 0; i < unassigned_tracks.size(); i++) {
      int t = unassigned_tracks[i];
      
      for (size_t j = 0; j < unassigned_detections.size(); j++) {
        int d = unassigned_detections[j];
        
        if (cost_matrix(t, d) < min_dist) {
          min_dist = cost_matrix(t, d);
          min_track = t;
          min_detection = d;
        }
      }
    }
    
    // Check if minimum distance is within threshold
    if (min_dist <= max_distance_) {
      assignment[min_track] = min_detection;
      
      // Remove assigned track and detection
      unassigned_tracks.erase(std::remove(unassigned_tracks.begin(),
        unassigned_tracks.end(), min_track), unassigned_tracks.end());
      unassigned_detections.erase(std::remove(unassigned_detections.begin(),
        unassigned_detections.end(), min_detection), unassigned_detections.end());
    } else {
      // No more valid assignments
      break;
    }
  }
}

void MultiTargetTracker::createNewTracks(
  const DetectionArray& detections,
  const std::vector<int>& unassigned_detections,
  double timestamp)
{
  for (const auto& idx : unassigned_detections) {
    const auto& det = detections[idx];
    
    // Create new track
    Track track;
    track.id = next_id_++;
    track.age = 1;
    track.hits = 1;
    track.misses = 0;
    track.confidence = det.confidence;
    track.status = 0;  // New track
    track.frame_id = det.header.frame_id;
    
    // Initialize Kalman filter with enhanced parameters
    KalmanFilter::MeasurementVector z;
    z << det.position.x, det.position.y, det.position.z;
    track.kf.init(z, timestamp);
    
    // Apply enhanced Kalman filter parameters for increased effectiveness
    if (!kalman_parameters_.empty()) {
      track.kf.setParameters(kalman_parameters_);
    }
    
    tracks_.push_back(track);
  }
}

void MultiTargetTracker::updateTracks(
  const DetectionArray& detections,
  const std::map<int, int>& assignment,
  const std::vector<int>& unassigned_tracks,
  double timestamp)
{
  (void)timestamp; // Suppress unused parameter warning
  
  // Update assigned tracks
  for (const auto& [track_idx, det_idx] : assignment) {
    const auto& det = detections[det_idx];
    auto& track = tracks_[track_idx];
    
    // Update Kalman filter with new measurement
    KalmanFilter::MeasurementVector z;
    z << det.position.x, det.position.y, det.position.z;
    track.kf.update(z);
    
    // Update track metadata
    track.hits++;
    track.age++;
    track.misses = 0;
    track.confidence = det.confidence;
    
    // Update status based on hits
    if (track.hits >= min_hits_) {
      track.status = 1;  // Confirmed track
    }
  }
  
  // Update unassigned tracks
  for (const auto& track_idx : unassigned_tracks) {
    auto& track = tracks_[track_idx];
    
    track.misses++;
    track.age++;
    
    // Update status based on misses
    if (track.misses > 3 && track.misses <= max_age_ / 2) {
      track.status = 2;  // Occluded
    } else if (track.misses > max_age_ / 2) {
      track.status = 3;  // Lost
    }
  }
}

void MultiTargetTracker::removeOldTracks()
{
  auto it = std::remove_if(tracks_.begin(), tracks_.end(),
    [this](const Track& track) {
      // Remove if track is too old or unconfirmed and missed for a while
      return (track.age > max_age_) ||
             (track.hits < min_hits_ && track.misses > min_hits_);
    });
  
  tracks_.erase(it, tracks_.end());
}

MultiTargetTracker::TrackedHumanArray MultiTargetTracker::tracksToMsg()
{
  TrackedHumanArray tracked_humans;
  
  for (const auto& track : tracks_) {
    human_tracker_ros2::msg::TrackedHuman msg;
    
    // Fill header
    msg.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    msg.header.frame_id = track.frame_id;
    
    // Track info
    msg.track_id = track.id;
    msg.track_age = track.age;
    msg.track_status = track.status;
    msg.confidence = track.confidence;
    
    // Position from Kalman filter state
    const auto& state = track.kf.getState();
    msg.pose.position.x = state(0);
    msg.pose.position.y = state(1);
    msg.pose.position.z = state(2);
    
    // Orientation (not estimated, set to identity)
    msg.pose.orientation.w = 1.0;
    
    // Velocity
    msg.velocity.x = state(3);
    msg.velocity.y = state(4);
    msg.velocity.z = state(5);
    
    // Covariance matrices
    Eigen::Matrix3d pos_cov = track.kf.getPositionCovariance();
    Eigen::Matrix3d vel_cov = track.kf.getVelocityCovariance();
    
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        msg.position_covariance[i*3+j] = pos_cov(i, j);
        msg.velocity_covariance[i*3+j] = vel_cov(i, j);
      }
    }
    
    tracked_humans.push_back(msg);
  }
  
  return tracked_humans;
}

} // namespace human_tracker_ros2