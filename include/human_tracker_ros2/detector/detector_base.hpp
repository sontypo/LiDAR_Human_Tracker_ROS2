#ifndef HUMAN_TRACKER_ROS2_DETECTOR_BASE_HPP
#define HUMAN_TRACKER_ROS2_DETECTOR_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <human_tracker_ros2/msg/detected_human.hpp>
#include <vector>

namespace human_tracker_ros2
{

class DetectorBase
{
public:
  using DetectionArray = std::vector<human_tracker_ros2::msg::DetectedHuman>;
  
  DetectorBase() = default;
  virtual ~DetectorBase() = default;
  
  // Main detection function to be implemented by derived classes
  virtual DetectionArray detect() = 0;
  
  // Set detection parameters
  virtual void setParameters(const std::vector<rclcpp::Parameter> & parameters) = 0;

protected:
  // Common detector parameters
  float confidence_threshold_ = 0.5f;
  std::string frame_id_ = "base_link";
};

} // namespace human_tracker_ros2

#endif // HUMAN_TRACKER_ROS2_DETECTOR_BASE_HPP