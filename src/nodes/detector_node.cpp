#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include "human_tracker_ros2/msg/detected_human.hpp"
#include "human_tracker_ros2/detector/lidar_detector.hpp"

namespace human_tracker_ros2
{

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode()
  : Node("human_detector_node")
  {
    // Declare parameters
    declareParameters();
    
    // Initialize detectors
    lidar_detector_ = std::make_shared<LidarDetector>();
    
    // Pass parameters to detectors
    std::vector<rclcpp::Parameter> params = this->get_parameters({
      "confidence_threshold",
      "cluster_tolerance",
      "min_cluster_size",
      "max_cluster_size",
      "min_height",
      "max_height",
      "person_width",
      "max_detection_range",
      "min_detection_range",
      "ground_tolerance",
      "outlier_threshold",
      "min_point_density",
      "aspect_ratio_threshold",
      "symmetry_threshold",
      "voxel_leaf_size",
      "adaptive_clustering",
      "standing_density_factor",
      "min_stationary_points"
    });
    lidar_detector_->setParameters(params);
    
    // Setup publishers
    detection_pub_ = create_publisher<human_tracker_ros2::msg::DetectedHuman>(
      "human_detections", 10);
    
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "human_detection_markers", 10);
      
    human_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "human_pointclouds", 10);
    
    // Setup subscribers
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/pointcloud", 10,
      std::bind(&DetectorNode::pointCloudCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Subscribed to topic: %s", point_cloud_sub_->get_topic_name());
    
    // Setup timer for detection processing
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DetectorNode::detectHumans, this));
      
    RCLCPP_INFO(get_logger(), "Human detector node initialized");
  }

private:
  void declareParameters()
  {
    // Basic detection parameters
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("cluster_tolerance", 0.2);
    declare_parameter("min_cluster_size", 30);
    declare_parameter("max_cluster_size", 10000);
    declare_parameter("min_height", 0.5);
    declare_parameter("max_height", 2.0);
    declare_parameter("person_width", 0.5);
    
    // Enhanced robust detection parameters
    declare_parameter("max_detection_range", 15.0);
    declare_parameter("min_detection_range", 0.5);
    declare_parameter("ground_tolerance", 0.15);
    declare_parameter("outlier_threshold", 1.0);
    declare_parameter("min_point_density", 5.0);         // Reduced for standing humans
    declare_parameter("aspect_ratio_threshold", 3.0);
    declare_parameter("symmetry_threshold", 0.2);        // Reduced for static poses
    
    // Standing human specific parameters
    declare_parameter("voxel_leaf_size", 0.08);          // Larger voxel size
    declare_parameter("adaptive_clustering", true);      // Enable adaptive clustering
    declare_parameter("standing_density_factor", 0.7);   // Density reduction factor
    declare_parameter("min_stationary_points", 20);      // Minimum points for standing humans
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Received point cloud with %d points", msg->width * msg->height);
    lidar_detector_->processPointCloud(msg);
  }
  
  void detectHumans()
  {
    // Run detection on latest data
    auto detections = lidar_detector_->detect();
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Detected %zu humans", detections.size());
    
    // Get human pointclouds from detector
    auto human_clouds = lidar_detector_->getHumanPointClouds();
    
    // Publish detections
    for (const auto& detection : detections) {
      detection_pub_->publish(detection);
    }
    
    // Publish human pointclouds
    publishHumanPointClouds(human_clouds, detections);
    
    // Create and publish visualization markers
    publishDetectionMarkers(detections);
    
    RCLCPP_DEBUG(get_logger(), "Published %zu human detections", detections.size());
  }

private:
  void publishDetectionMarkers(const std::vector<human_tracker_ros2::msg::DetectedHuman>& detections)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "livox_frame";  // Use the same frame as detections
    clear_marker.header.stamp = now();
    clear_marker.ns = "human_detections";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Create markers for each detection
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto& detection = detections[i];
      
      // Create cylinder marker for human body
      visualization_msgs::msg::Marker body_marker;
      body_marker.header = detection.header;
      body_marker.ns = "human_detections";
      body_marker.id = detection.detection_id * 2;  // Use even IDs for body
      body_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      body_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position
      body_marker.pose.position = detection.position;
      body_marker.pose.position.z += detection.dimensions.z / 2.0;  // Center at half height
      body_marker.pose.orientation.w = 1.0;
      
      // Scale (use detection dimensions)
      body_marker.scale.x = detection.dimensions.x;  // width
      body_marker.scale.y = detection.dimensions.y;  // depth
      body_marker.scale.z = detection.dimensions.z;  // height
      
      // Color based on confidence (green = high confidence, red = low confidence)
      body_marker.color.r = 1.0 - detection.confidence;
      body_marker.color.g = detection.confidence;
      body_marker.color.b = 0.2;
      body_marker.color.a = 0.7;  // Semi-transparent
      
      // Lifetime
      body_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      
      marker_array.markers.push_back(body_marker);
      
      // Create text marker for detection ID and confidence
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = detection.header;
      text_marker.ns = "human_detections";
      text_marker.id = detection.detection_id * 2 + 1;  // Use odd IDs for text
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position above the detection
      text_marker.pose.position = detection.position;
      text_marker.pose.position.z += detection.dimensions.z + 0.3;  // 30cm above
      text_marker.pose.orientation.w = 1.0;
      
      // Scale
      text_marker.scale.z = 0.3;  // Text size
      
      // Color (white text)
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      
      // Text content
      text_marker.text = "ID:" + std::to_string(detection.detection_id) + 
                        " (" + std::to_string(static_cast<int>(detection.confidence * 100)) + "%)";
      
      // Lifetime
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      
      marker_array.markers.push_back(text_marker);
    }
    
    // Publish markers
    marker_pub_->publish(marker_array);
  }
  
  void publishHumanPointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& human_clouds, 
                              const std::vector<human_tracker_ros2::msg::DetectedHuman>& detections)
  {
    if (human_clouds.size() != detections.size()) {
      RCLCPP_WARN(get_logger(), "Mismatch between human clouds (%zu) and detections (%zu)", 
                  human_clouds.size(), detections.size());
      return;
    }
    
    for (size_t i = 0; i < human_clouds.size(); ++i) {
      if (human_clouds[i] && !human_clouds[i]->empty()) {
        // Create a colored pointcloud for better visualization
        pcl::PointCloud<pcl::PointXYZI>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        colored_cloud->reserve(human_clouds[i]->size());
        
        // Calculate distance from detection center for intensity gradation
        auto& detection = detections[i];
        Eigen::Vector3f center(detection.position.x, detection.position.y, detection.position.z);
        
        // Assign unique color/intensity based on detection ID and distance from center
        float base_intensity = 50.0f + (detection.detection_id % 6) * 40.0f; // Range: 50-250
        float max_distance = std::max({detection.dimensions.x, detection.dimensions.y, detection.dimensions.z}) / 2.0f;
        
        for (const auto& point : human_clouds[i]->points) {
          pcl::PointXYZI colored_point;
          colored_point.x = point.x;
          colored_point.y = point.y;
          colored_point.z = point.z;
          
          // Calculate distance from cluster center
          Eigen::Vector3f point_pos(point.x, point.y, point.z);
          float distance = (point_pos - center).norm();
          
          // Create intensity gradient: higher intensity at center, lower at edges
          float distance_factor = std::max(0.0f, 1.0f - (distance / max_distance));
          colored_point.intensity = base_intensity + distance_factor * 100.0f; // Range: base to base+100
          
          colored_cloud->push_back(colored_point);
        }
        
        // Convert PCL cloud to ROS message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*colored_cloud, cloud_msg);
        
        // Set frame and timestamp
        cloud_msg.header.frame_id = "livox_frame";
        cloud_msg.header.stamp = this->get_clock()->now();
        
        // Publish the colored human pointcloud
        human_cloud_pub_->publish(cloud_msg);
        
        RCLCPP_DEBUG(get_logger(), "Published colored pointcloud for human %u with %zu points (intensity: %.1f)", 
                    detections[i].detection_id, colored_cloud->size(), base_intensity);
      }
    }
  }
  
  // Detectors
  std::shared_ptr<LidarDetector> lidar_detector_;
  
  // Publishers
  rclcpp::Publisher<human_tracker_ros2::msg::DetectedHuman>::SharedPtr detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_cloud_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace human_tracker_ros2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<human_tracker_ros2::DetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}