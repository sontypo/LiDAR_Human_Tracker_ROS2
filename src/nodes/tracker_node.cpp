#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "human_tracker_ros2/msg/detected_human.hpp"
#include "human_tracker_ros2/msg/tracked_human.hpp"
#include "human_tracker_ros2/msg/tracked_humans.hpp"
#include "human_tracker_ros2/tracker/multi_target_tracker.hpp"

namespace human_tracker_ros2
{

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode()
  : Node("human_tracker_node")
  {
    // Declare parameters
    declareParameters();
    
    // Initialize tracker
    tracker_ = std::make_shared<MultiTargetTracker>();
    
    // Pass parameters to tracker (including enhanced Kalman filter parameters)
    std::vector<rclcpp::Parameter> params = this->get_parameters({
      "max_distance",
      "max_age", 
      "min_hits",
      "kalman_process_noise_pos",
      "kalman_process_noise_vel",
      "kalman_process_noise_acc",
      "kalman_base_measurement_noise",
      "kalman_max_measurement_noise",
      "kalman_innovation_threshold",
      "kalman_initial_position_uncertainty",
      "kalman_initial_velocity_uncertainty",
      "kalman_initial_acceleration_uncertainty",
      "kalman_prediction_uncertainty_growth",
      "kalman_innovation_forgetting_factor",
      "kalman_motion_smoothing_factor",
      "kalman_max_human_velocity",
      "kalman_max_human_acceleration",
      "kalman_outlier_threshold",
      "kalman_enable_joseph_form",
      "kalman_enable_outlier_rejection",
      "kalman_enable_velocity_gating"
    });
    tracker_->setParameters(params);
    
    // Setup publishers
    tracked_humans_pub_ = create_publisher<human_tracker_ros2::msg::TrackedHumans>(
      "tracked_humans", 10);
    
    tracked_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "tracked_humans_markers", 10);
    
    // Setup subscribers
    detection_sub_ = create_subscription<human_tracker_ros2::msg::DetectedHuman>(
      "human_detections", 100,
      std::bind(&TrackerNode::detectionCallback, this, std::placeholders::_1));
    
    // Setup timer for processing detections
    double process_rate = get_parameter("process_rate").as_double();
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / process_rate));
    timer_ = create_wall_timer(period, std::bind(&TrackerNode::processDetections, this));
      
    RCLCPP_INFO(get_logger(), "Human tracker node initialized");
  }

private:
  void declareParameters()
  {
    // Basic tracking parameters
    declare_parameter("max_distance", 1.0);
    declare_parameter("max_age", 10);
    declare_parameter("min_hits", 3);
    declare_parameter("process_rate", 10.0);  // Hz
    
    // Enhanced Kalman Filter Parameters for Maximum Effectiveness
    declare_parameter("kalman_process_noise_pos", 0.3);
    declare_parameter("kalman_process_noise_vel", 0.8);
    declare_parameter("kalman_process_noise_acc", 1.5);
    declare_parameter("kalman_base_measurement_noise", 0.08);
    declare_parameter("kalman_max_measurement_noise", 0.8);
    declare_parameter("kalman_innovation_threshold", 1.8);
    declare_parameter("kalman_initial_position_uncertainty", 50.0);
    declare_parameter("kalman_initial_velocity_uncertainty", 10.0);
    declare_parameter("kalman_initial_acceleration_uncertainty", 100.0);
    declare_parameter("kalman_prediction_uncertainty_growth", 0.3);
    declare_parameter("kalman_innovation_forgetting_factor", 0.85);
    declare_parameter("kalman_motion_smoothing_factor", 0.95);
    declare_parameter("kalman_max_human_velocity", 3.0);
    declare_parameter("kalman_max_human_acceleration", 5.0);
    declare_parameter("kalman_outlier_threshold", 3.0);
    declare_parameter("kalman_enable_joseph_form", true);
    declare_parameter("kalman_enable_outlier_rejection", true);
    declare_parameter("kalman_enable_velocity_gating", true);
  }

  void detectionCallback(const human_tracker_ros2::msg::DetectedHuman::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Received detection with ID %d", msg->detection_id);
    std::lock_guard<std::mutex> lock(detections_mutex_);
    detections_.push_back(*msg);
  }
  
  void processDetections()
  {
    // Get current detections
    std::vector<human_tracker_ros2::msg::DetectedHuman> current_detections;
    {
      std::lock_guard<std::mutex> lock(detections_mutex_);
      current_detections = std::move(detections_);
      detections_.clear();
    }
    
    // Skip if no detections
    if (current_detections.empty()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "No detections to process");
      return;
    }
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Processing %zu detections", current_detections.size());
    
    // Update tracker with current detections
    double timestamp = rclcpp::Clock().now().seconds();
    auto tracked_humans = tracker_->update(current_detections, timestamp);
    
    // Publish tracked humans
    human_tracker_ros2::msg::TrackedHumans msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "livox_frame";  // Use appropriate frame
    msg.humans = tracked_humans;
    msg.frame_id = "livox_frame";
    
    tracked_humans_pub_->publish(msg);
    
    // Create and publish visualization markers
    publishTrackedMarkers(tracked_humans);
    
    RCLCPP_DEBUG(get_logger(), "Published %zu tracked humans", tracked_humans.size());
  }

private:
  void publishTrackedMarkers(const std::vector<human_tracker_ros2::msg::TrackedHuman>& tracked_humans)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "livox_frame";
    clear_marker.header.stamp = now();
    clear_marker.ns = "tracked_humans";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Create markers for each tracked human
    for (const auto& human : tracked_humans) {
      
      // Create arrow marker for movement direction and velocity
      visualization_msgs::msg::Marker arrow_marker;
      arrow_marker.header.frame_id = "livox_frame";
      arrow_marker.header.stamp = now();
      arrow_marker.ns = "tracked_humans";
      arrow_marker.id = human.track_id * 3;  // Use multiples of 3 for arrows
      arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
      arrow_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position and orientation
      arrow_marker.pose.position = human.pose.position;
      arrow_marker.pose.position.z += 0.5;  // Raise above ground
      
      // Calculate orientation from velocity
      double yaw = std::atan2(human.velocity.y, human.velocity.x);
      arrow_marker.pose.orientation.z = std::sin(yaw * 0.5);
      arrow_marker.pose.orientation.w = std::cos(yaw * 0.5);
      
      // Scale based on velocity magnitude
      double velocity_magnitude = std::sqrt(human.velocity.x * human.velocity.x + 
                                          human.velocity.y * human.velocity.y);
      arrow_marker.scale.x = std::max(0.3, velocity_magnitude * 0.5);  // Length
      arrow_marker.scale.y = 0.1;  // Width
      arrow_marker.scale.z = 0.1;  // Height
      
      // Color based on track age (blue = new, purple = old)
      arrow_marker.color.r = 0.2;
      arrow_marker.color.g = 0.2;
      arrow_marker.color.b = 1.0;
      arrow_marker.color.a = 0.8;
      
      // Lifetime
      arrow_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      marker_array.markers.push_back(arrow_marker);
      
      // Create sphere marker for tracked position
      visualization_msgs::msg::Marker sphere_marker;
      sphere_marker.header.frame_id = "livox_frame";
      sphere_marker.header.stamp = now();
      sphere_marker.ns = "tracked_humans";
      sphere_marker.id = human.track_id * 3 + 1;  // Use multiples of 3 + 1 for spheres
      sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
      sphere_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position
      sphere_marker.pose.position = human.pose.position;
      sphere_marker.pose.position.z += 1.0;  // At human center height
      sphere_marker.pose.orientation.w = 1.0;
      
      // Scale
      sphere_marker.scale.x = 0.4;
      sphere_marker.scale.y = 0.4;
      sphere_marker.scale.z = 0.4;
      
      // Color (green for tracked humans)
      sphere_marker.color.r = 0.0;
      sphere_marker.color.g = 1.0;
      sphere_marker.color.b = 0.0;
      sphere_marker.color.a = 0.8;
      
      // Lifetime
      sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      marker_array.markers.push_back(sphere_marker);
      
      // Create text marker for track information
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "livox_frame";
      text_marker.header.stamp = now();
      text_marker.ns = "tracked_humans";
      text_marker.id = human.track_id * 3 + 2;  // Use multiples of 3 + 2 for text
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position above the tracked human
      text_marker.pose.position = human.pose.position;
      text_marker.pose.position.z += 2.2;  // Above the human
      text_marker.pose.orientation.w = 1.0;
      
      // Scale
      text_marker.scale.z = 0.4;  // Text size
      
      // Color (bright yellow text)
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0;
      text_marker.color.a = 1.0;
      
      // Text content with track ID and velocity
      text_marker.text = "T" + std::to_string(human.track_id) + 
                        " v:" + std::to_string(static_cast<int>(velocity_magnitude * 100)) + "cm/s";
      
      // Lifetime
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      marker_array.markers.push_back(text_marker);
    }
    
    // Publish markers
    tracked_marker_pub_->publish(marker_array);
  }
  
  // Tracker
  std::shared_ptr<MultiTargetTracker> tracker_;
  
  // Publishers
  rclcpp::Publisher<human_tracker_ros2::msg::TrackedHumans>::SharedPtr tracked_humans_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_marker_pub_;
  
  // Subscribers
  rclcpp::Subscription<human_tracker_ros2::msg::DetectedHuman>::SharedPtr detection_sub_;
  
  // Detection buffer
  std::vector<human_tracker_ros2::msg::DetectedHuman> detections_;
  std::mutex detections_mutex_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace human_tracker_ros2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<human_tracker_ros2::TrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}