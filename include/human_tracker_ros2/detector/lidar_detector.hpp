#ifndef HUMAN_TRACKER_ROS2_LIDAR_DETECTOR_HPP
#define HUMAN_TRACKER_ROS2_LIDAR_DETECTOR_HPP

#include "detector_base.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

namespace human_tracker_ros2
{

class LidarDetector : public DetectorBase
{
public:
  explicit LidarDetector();
  ~LidarDetector() override = default;
  
  // Implementation of the detect method for LiDAR data
  DetectionArray detect() override;
  
  // Set LiDAR-specific parameters
  void setParameters(const std::vector<rclcpp::Parameter> & parameters) override;
  
  // Process incoming point cloud
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // Get pointclouds for detected humans
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getHumanPointClouds() const;

private:
  // Point cloud segmentation and clustering methods
  void segmentGround(const sensor_msgs::msg::PointCloud2 & cloud);
  std::vector<sensor_msgs::msg::PointCloud2> clusterPointCloud();
  
  // Human classification from point cloud clusters
  float classifyHumanLikelihood(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
  
  // Enhanced classification methods for robust detection
  float calculateShapeComplexity(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
  float calculateVerticalDistribution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
  float calculateDensityMetrics(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
  float calculateSymmetryScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
  bool checkTemporalConsistency(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
  
  // Advanced filtering methods
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterByROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr input);

  // Latest point cloud data
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::mutex cloud_mutex_;
  
  // Store human pointclouds from latest detection
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> human_pointclouds_;
  std::mutex human_clouds_mutex_;
  
  // Temporal tracking for consistency
  struct ClusterHistory {
    pcl::PointXYZ centroid;
    double timestamp;
    float confidence;
    int tracking_id;
  };
  std::vector<ClusterHistory> cluster_history_;
  std::mutex history_mutex_;
  double max_history_age_ = 2.0;  // seconds
  
  // LiDAR-specific parameters
  float cluster_tolerance_ = 0.2;
  int min_cluster_size_ = 30;
  int max_cluster_size_ = 10000;
  float min_height_ = 0.5;
  float max_height_ = 2.0;
  float person_width_ = 0.5;
  
  // Enhanced detection parameters
  float max_detection_range_ = 15.0;   // Maximum detection distance
  float min_detection_range_ = 0.5;    // Minimum detection distance
  float ground_tolerance_ = 0.15;      // Ground plane tolerance
  float outlier_threshold_ = 1.0;      // Statistical outlier removal threshold
  float min_point_density_ = 5.0;      // Points per cubic meter (reduced for standing humans)
  float aspect_ratio_threshold_ = 3.0; // Max height/width ratio
  float symmetry_threshold_ = 0.2;     // Symmetry score threshold (reduced for static poses)
  
  // Stationary detection parameters
  float voxel_leaf_size_ = 0.08f;      // Larger voxel size to preserve standing humans
  bool adaptive_clustering_ = true;     // Enable adaptive clustering for standing humans
  float standing_density_factor_ = 0.7f; // Density reduction factor for standing humans
  int min_stationary_points_ = 20;     // Minimum points for standing humans
  
  // Detection counter for ID assignment
  uint32_t detection_counter_ = 0;
};

} // namespace human_tracker_ros2

#endif // HUMAN_TRACKER_ROS2_LIDAR_DETECTOR_HPP