#include "human_tracker_ros2/detector/lidar_detector.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <algorithm>
#include <numeric>

namespace human_tracker_ros2
{

LidarDetector::LidarDetector()
{
  // Initialize detector
}

void LidarDetector::setParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "confidence_threshold") {
      confidence_threshold_ = param.as_double();
    } else if (param.get_name() == "cluster_tolerance") {
      cluster_tolerance_ = param.as_double();
    } else if (param.get_name() == "min_cluster_size") {
      min_cluster_size_ = param.as_int();
    } else if (param.get_name() == "max_cluster_size") {
      max_cluster_size_ = param.as_int();
    } else if (param.get_name() == "min_height") {
      min_height_ = param.as_double();
    } else if (param.get_name() == "max_height") {
      max_height_ = param.as_double();
    } else if (param.get_name() == "person_width") {
      person_width_ = param.as_double();
    } else if (param.get_name() == "max_detection_range") {
      max_detection_range_ = param.as_double();
    } else if (param.get_name() == "min_detection_range") {
      min_detection_range_ = param.as_double();
    } else if (param.get_name() == "ground_tolerance") {
      ground_tolerance_ = param.as_double();
    } else if (param.get_name() == "outlier_threshold") {
      outlier_threshold_ = param.as_double();
    } else if (param.get_name() == "min_point_density") {
      min_point_density_ = param.as_double();
    } else if (param.get_name() == "aspect_ratio_threshold") {
      aspect_ratio_threshold_ = param.as_double();
    } else if (param.get_name() == "symmetry_threshold") {
      symmetry_threshold_ = param.as_double();
    } else if (param.get_name() == "voxel_leaf_size") {
      voxel_leaf_size_ = param.as_double();
    } else if (param.get_name() == "adaptive_clustering") {
      adaptive_clustering_ = param.as_bool();
    } else if (param.get_name() == "standing_density_factor") {
      standing_density_factor_ = param.as_double();
    } else if (param.get_name() == "min_stationary_points") {
      min_stationary_points_ = param.as_int();
    }
  }
}

void LidarDetector::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
}

DetectorBase::DetectionArray LidarDetector::detect()
{
  DetectionArray detections;
  
  // Clear previous human pointclouds
  {
    std::lock_guard<std::mutex> lock(human_clouds_mutex_);
    human_pointclouds_.clear();
  }
  
  // Check if we have point cloud data
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!latest_cloud_) {
      return detections;
    }
    cloud = latest_cloud_;
  }
  
  // Convert to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  
  if (pcl_cloud->empty()) {
    return detections;
  }
  
  // Advanced preprocessing pipeline
  pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = preprocessPointCloud(pcl_cloud);
  
  if (processed_cloud->empty()) {
    return detections;
  }
  
  // Enhanced ground plane segmentation with better robustness
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ground_tolerance_);
  seg.setMaxIterations(1000);
  seg.setProbability(0.99);
  seg.setInputCloud(processed_cloud);
  seg.segment(*inliers, *coefficients);
  
  // Extract non-ground points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.setInputCloud(processed_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*non_ground_cloud);
  
  if (non_ground_cloud->empty()) {
    return detections;
  }
  
  // Adaptive clustering with multi-scale approach for standing humans
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(non_ground_cloud);
  
  // Use adaptive cluster tolerance and size for standing humans
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(std::min(min_cluster_size_, min_stationary_points_));  // Lower minimum for standing
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(non_ground_cloud);
  ec.extract(cluster_indices);
  
  // If no clusters found with normal parameters, try more sensitive settings
  if (cluster_indices.empty() && adaptive_clustering_) {
    ec.setClusterTolerance(cluster_tolerance_ * 1.5);  // Larger tolerance
    ec.setMinClusterSize(min_stationary_points_);      // Even smaller minimum
    ec.extract(cluster_indices);
  }
  
  // Process each cluster with enhanced classification
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto& idx : indices.indices) {
      cluster->push_back((*non_ground_cloud)[idx]);
    }
    
    // Skip clusters that are too small after extraction (more lenient for standing humans)
    if (cluster->size() < static_cast<size_t>(min_stationary_points_)) {
      continue;
    }
    
    // Compute cluster properties
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cluster);
    feature_extractor.compute();
    
    pcl::PointXYZ min_point, max_point;
    feature_extractor.getAABB(min_point, max_point);
    
    float height = max_point.z - min_point.z;
    float width = std::max(max_point.x - min_point.x, max_point.y - min_point.y);
    float depth = std::min(max_point.x - min_point.x, max_point.y - min_point.y);
    
    // More lenient dimensional filtering for standing humans
    bool size_check = (height >= min_height_ * 0.8 && height <= max_height_ && 
                      width <= person_width_ * 2.5 && width >= 0.15 &&    // More flexible width
                      depth <= person_width_ * 2.5 && depth >= 0.08);     // More flexible depth
    
    // More lenient aspect ratio check for standing humans (can be thinner)
    float aspect_ratio = height / std::max(width, 0.1f);
    bool aspect_check = (aspect_ratio >= 1.2 && aspect_ratio <= aspect_ratio_threshold_ * 1.2);
    
    // Distance-based filtering
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    float distance = std::sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1]);
    bool distance_check = (distance >= min_detection_range_ && distance <= max_detection_range_);
    
    if (!size_check || !aspect_check || !distance_check) {
      continue;
    }
    
    // Multi-criteria human classification
    float confidence = classifyHumanLikelihood(cluster);
    
    // Skip if confidence is below threshold
    if (confidence < confidence_threshold_) {
      continue;
    }
    
    // Additional robustness checks with standing human considerations
    float density = calculateDensityMetrics(cluster);
    float vertical_dist = calculateVerticalDistribution(cluster);
    float symmetry = calculateSymmetryScore(cluster);
    
    // Apply standing human density factor (standing humans may have lower density)
    float adjusted_density_threshold = min_point_density_ * standing_density_factor_;
    
    // Combined confidence adjustment with standing human considerations
    float final_confidence = confidence * 0.6 + 
                            std::min(density / adjusted_density_threshold, 1.0f) * 0.2 +
                            vertical_dist * 0.1 +
                            (symmetry > symmetry_threshold_ ? 0.1 : 0.05);  // More lenient symmetry
    
    final_confidence = std::max(0.0f, std::min(1.0f, final_confidence));
    
    // Lower confidence threshold for standing humans if they pass basic checks
    float effective_threshold = confidence_threshold_;
    if (cluster->size() >= static_cast<size_t>(min_stationary_points_) && 
        height >= min_height_ && aspect_ratio >= 1.2) {
      effective_threshold *= 0.8f;  // 20% lower threshold for likely standing humans
    }
    
    if (final_confidence < effective_threshold) {
      continue;
    }
    
    // Create detection
    human_tracker_ros2::msg::DetectedHuman human;
    human.header = cloud->header;
    human.detection_id = detection_counter_++;
    human.confidence = final_confidence;
    
    human.position.x = centroid[0];
    human.position.y = centroid[1];
    human.position.z = centroid[2];
    
    human.dimensions.x = width;
    human.dimensions.y = depth;
    human.dimensions.z = height;
    
    human.detection_source = "lidar_enhanced";
    
    detections.push_back(human);
    
    // Store human pointcloud
    {
      std::lock_guard<std::mutex> lock(human_clouds_mutex_);
      human_pointclouds_.push_back(cluster);
    }
    
    // Update cluster history for temporal consistency
    {
      std::lock_guard<std::mutex> lock(history_mutex_);
      ClusterHistory history;
      history.centroid = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
      history.timestamp = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
      history.confidence = final_confidence;
      history.tracking_id = human.detection_id;
      cluster_history_.push_back(history);
      
      // Clean old history
      auto current_time = history.timestamp;
      cluster_history_.erase(
        std::remove_if(cluster_history_.begin(), cluster_history_.end(),
          [this, current_time](const ClusterHistory& h) {
            return (current_time - h.timestamp) > max_history_age_;
          }),
        cluster_history_.end());
    }
  }
  
  return detections;
}

float LidarDetector::classifyHumanLikelihood(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  if (!cluster || cluster->empty()) {
    return 0.0f;
  }

  // Get cluster dimensions using PCA for more accurate measurements
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  
  // Get bounding box for traditional measurements
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cluster);
  feature_extractor.compute();
  
  pcl::PointXYZ min_point, max_point;
  feature_extractor.getAABB(min_point, max_point);
  
  float height = max_point.z - min_point.z;
  float width = std::max(max_point.x - min_point.x, max_point.y - min_point.y);
  float depth = std::min(max_point.x - min_point.x, max_point.y - min_point.y);
  
  // Multi-criteria human likelihood assessment
  
  // 1. Dimensional similarity to human (40% weight)
  float ideal_height = 1.7f;  // Average human height
  float ideal_width = 0.5f;   // Average human width
  float ideal_depth = 0.3f;   // Average human depth
  
  float height_score = std::exp(-std::pow((height - ideal_height) / 0.3f, 2));
  float width_score = std::exp(-std::pow((width - ideal_width) / 0.2f, 2));
  float depth_score = std::exp(-std::pow((depth - ideal_depth) / 0.15f, 2));
  
  float dimensional_score = (height_score * 0.5f + width_score * 0.3f + depth_score * 0.2f);
  
  // 2. Shape complexity (20% weight) - humans have moderate complexity
  float complexity_score = calculateShapeComplexity(cluster);
  
  // 3. Vertical distribution (20% weight) - humans have characteristic vertical structure
  float vertical_score = calculateVerticalDistribution(cluster);
  
  // 4. Point density (10% weight) - humans have relatively consistent density
  float density_score = calculateDensityMetrics(cluster);
  
  // 5. Symmetry (10% weight) - humans have some bilateral symmetry
  float symmetry_score = calculateSymmetryScore(cluster);
  
  // Combine all scores
  float confidence = dimensional_score * 0.4f +
                    complexity_score * 0.2f +
                    vertical_score * 0.2f +
                    density_score * 0.1f +
                    symmetry_score * 0.1f;
  
  // Apply distance-based confidence adjustment (closer objects are more reliable)
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);
  float distance = std::sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1]);
  float distance_factor = std::max(0.5f, std::min(1.0f, (max_detection_range_ - distance) / max_detection_range_));
  
  confidence *= distance_factor;
  
  return std::max(0.0f, std::min(1.0f, confidence));
}

float LidarDetector::calculateShapeComplexity(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  if (cluster->size() < 10) return 0.0f;
  
  // Calculate normalized eigenvalues to assess shape complexity
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  
  // Normalize eigenvalues
  float total = eigenvalues.sum();
  if (total < 1e-6) return 0.0f;
  
  Eigen::Vector3f normalized = eigenvalues / total;
  
  // Calculate planarity and linearity measures
  float planarity = (normalized[1] - normalized[2]) / normalized[0];
  float linearity = (normalized[0] - normalized[1]) / normalized[0];
  
  // Humans typically have moderate complexity (not too planar, not too linear)
  float ideal_planarity = 0.3f;
  float ideal_linearity = 0.4f;
  
  float planarity_score = 1.0f - std::abs(planarity - ideal_planarity) / ideal_planarity;
  float linearity_score = 1.0f - std::abs(linearity - ideal_linearity) / ideal_linearity;
  
  return std::max(0.0f, (planarity_score + linearity_score) / 2.0f);
}

float LidarDetector::calculateVerticalDistribution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  if (cluster->size() < 5) return 0.0f;
  
  // Find height range
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  
  for (const auto& point : cluster->points) {
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  
  float height = max_z - min_z;
  if (height < 0.1f) return 0.0f;
  
  // Divide into vertical segments and count points
  const int num_segments = 5;
  std::vector<int> segment_counts(num_segments, 0);
  
  for (const auto& point : cluster->points) {
    int segment = static_cast<int>((point.z - min_z) / height * (num_segments - 1));
    segment = std::max(0, std::min(num_segments - 1, segment));
    segment_counts[segment]++;
  }
  
  // Calculate distribution uniformity (humans have relatively uniform vertical distribution)
  float mean_count = static_cast<float>(cluster->size()) / num_segments;
  float variance = 0.0f;
  
  for (int count : segment_counts) {
    variance += std::pow(count - mean_count, 2);
  }
  variance /= num_segments;
  
  // Lower variance indicates more uniform distribution (better for humans)
  float uniformity = 1.0f / (1.0f + variance / (mean_count * mean_count));
  
  return std::max(0.0f, std::min(1.0f, uniformity));
}

float LidarDetector::calculateDensityMetrics(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  if (cluster->size() < 5) return 0.0f;
  
  // Calculate cluster volume using bounding box (more forgiving for standing humans)
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cluster);
  feature_extractor.compute();
  
  pcl::PointXYZ min_point, max_point;
  feature_extractor.getAABB(min_point, max_point);
  
  float volume = (max_point.x - min_point.x) * 
                 (max_point.y - min_point.y) * 
                 (max_point.z - min_point.z);
  
  if (volume < 1e-6) return 0.0f;
  
  float density = cluster->size() / volume;
  
  // More forgiving normalization for standing humans (5-40 points per cubic meter)
  float normalized_density = std::min(1.0f, density / 40.0f);
  
  // Boost score for reasonable density ranges typical of standing humans
  if (density >= 5.0f && density <= 25.0f) {
    normalized_density = std::min(1.0f, normalized_density * 1.2f);
  }
  
  return normalized_density;
}

float LidarDetector::calculateSymmetryScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  if (cluster->size() < 10) return 0.0f;
  
  // Calculate centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);
  
  // Simple bilateral symmetry check along Y-axis (left-right)
  std::vector<float> left_points, right_points;
  
  for (const auto& point : cluster->points) {
    if (point.y < centroid[1]) {
      left_points.push_back(centroid[1] - point.y);  // Distance from center
    } else {
      right_points.push_back(point.y - centroid[1]);
    }
  }
  
  if (left_points.empty() || right_points.empty()) return 0.0f;
  
  // Calculate mean distances
  float left_mean = std::accumulate(left_points.begin(), left_points.end(), 0.0f) / left_points.size();
  float right_mean = std::accumulate(right_points.begin(), right_points.end(), 0.0f) / right_points.size();
  
  // Calculate symmetry score based on balance
  float balance_ratio = std::min(left_points.size(), right_points.size()) / 
                       static_cast<float>(std::max(left_points.size(), right_points.size()));
  
  float distance_similarity = 1.0f - std::abs(left_mean - right_mean) / std::max(left_mean + right_mean, 0.1f);
  
  return (balance_ratio + distance_similarity) / 2.0f;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarDetector::preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  // Step 1: Region of Interest filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_filtered = filterByROI(input);
  
  // Step 2: Gentler voxel grid downsampling (preserve more points for standing humans)
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(roi_filtered);
  vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);  // Larger voxel size
  vg.filter(*downsampled);
  
  // Step 3: More conservative outlier removal for standing humans
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud = removeOutliers(downsampled);
  
  return clean_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarDetector::removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input);
  sor.setMeanK(15);  // Reduced neighbors for gentler filtering
  sor.setStddevMulThresh(outlier_threshold_ * 1.5);  // More lenient threshold for standing humans
  sor.filter(*filtered);
  
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarDetector::filterByROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Filter by detection range
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(input);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-max_detection_range_, max_detection_range_);
  pass_x.filter(*filtered);
  
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-max_detection_range_, max_detection_range_);
  pass_y.filter(*filtered);
  
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(filtered);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-0.5, 3.0);  // Reasonable height range for human detection
  pass_z.filter(*filtered);
  
  return filtered;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> LidarDetector::getHumanPointClouds() const
{
  std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(human_clouds_mutex_));
  return human_pointclouds_;
}

} // namespace human_tracker_ros2