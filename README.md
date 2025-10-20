# LiDAR Human Tracker ROS2
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

> A comprehensive ROS2 package for real-time human detection and tracking in outdoor environments using 3D LiDAR sensors (only). This package provides enterprise-grade tracking capabilities with advanced Kalman filter enhancements, robust multi-criteria detection, and specialized support for standing human detection.

## 🚀 Features

### Advanced Detection System
- **Multi-criteria Human Classification**: 5-stage validation pipeline including shape complexity, density metrics, symmetry analysis, and temporal consistency
- **Standing Human Detection**: Specialized algorithms and parameters optimized for stationary human detection
- **Robust Clustering**: Adaptive clustering with statistical outlier removal and ground plane segmentation
- **Colored Pointcloud Publishing**: Individual human pointclouds with unique intensity gradients for visualization

### Enhanced Kalman Filter Tracking
- **9-State Constant Acceleration Model**: Position, velocity, and acceleration tracking in 3D space
- **20+ Configurable Parameters**: Comprehensive parameter system for maximum tracking effectiveness
- **Outlier Rejection System**: Mahalanobis distance validation with adaptive measurement noise
- **Motion Constraints**: Physical human motion model validation and velocity gating
- **Joseph Form Updates**: Numerically stable covariance updates for enhanced precision
- **Tracking Quality Metrics**: Real-time tracking quality scoring and performance monitoring

### Real-World Robustness
- **Adaptive Parameter System**: Real-time tuning for different environmental conditions
- **Multi-Target Tracking**: Simultaneous tracking of multiple humans with data association
- **Standing Detection Optimization**: Specialized handling for stationary humans
- **Environmental Adaptability**: Configurable parameters for various outdoor scenarios

## 📋 Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Topics and Services](#topics-and-services)
- [Parameters](#parameters)
- [Visualization](#visualization)
- [Performance Tuning](#performance-tuning)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## 🛠 Installation

### Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **PCL (Point Cloud Library)** >= 1.12
- **Eigen3** >= 3.3
- **LiDAR sensor** (tested with Livox sensors)

### Dependencies

```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-pcl-conversions
sudo apt install ros-humble-visualization-msgs
sudo apt install libpcl-dev
sudo apt install libeigen3-dev
```

### Build Instructions

1. **Clone the repository**:
```bash
cd ~/your_ros2_ws/src
git clone <repository-url> human_tracker_ros2
```

2. **Install dependencies**:
```bash
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package**:
```bash
colcon build --packages-select human_tracker_ros2
source install/setup.bash
```

## 🎯 Usage

### Launch Arguments

The launch file supports the following configurable arguments:

| Argument | Default Value | Description |
|----------|---------------|-------------|
| `lidar_topic` | `/livox/lidar` | LiDAR point cloud topic name |
| `use_sim_time` | `false` | Use simulation time if true |

**Examples**:
```bash
# Use with Velodyne LiDAR
ros2 launch human_tracker_ros2 human_tracker.launch.py lidar_topic:=/velodyne_points

# Use with Ouster LiDAR  
ros2 launch human_tracker_ros2 human_tracker.launch.py lidar_topic:=/ouster/points

# Use with simulation
ros2 launch human_tracker_ros2 human_tracker.launch.py lidar_topic:=/scan_points use_sim_time:=true
```

### Quick Start

1. **Launch the complete system**:
```bash
ros2 launch human_tracker_ros2 human_tracker.launch.py
```

2. **Launch with custom parameters**:
```bash
ros2 launch human_tracker_ros2 human_tracker.launch.py use_sim_time:=true
```

3. **Launch with custom LiDAR topic**:
```bash
ros2 launch human_tracker_ros2 human_tracker.launch.py lidar_topic:=/your/lidar/topic
```

4. **Launch with multiple custom parameters**:
```bash
ros2 launch human_tracker_ros2 human_tracker.launch.py lidar_topic:=/velodyne_points use_sim_time:=true
```

### Individual Nodes

**Detector Node**:
```bash
ros2 run human_tracker_ros2 detector_node --ros-args --params-file config/detector_params.yaml
```

**Tracker Node**:
```bash
ros2 run human_tracker_ros2 tracker_node --ros-args --params-file config/tracker_params.yaml
```

### Visualization in RViz

1. **Launch RViz**:
```bash
rviz2
```

2. **Add displays**:
   - **PointCloud2**: `/human_detections_pointcloud` (colored individual humans)
   - **MarkerArray**: `/tracked_humans_markers` (tracking visualization)
   - **PointCloud2**: Your LiDAR topic (e.g., `/livox/lidar`, `/velodyne_points`, etc.)

## ⚙️ Configuration

### Detector Parameters (`config/detector_params.yaml`)

#### Basic Detection
```yaml
confidence_threshold: 0.5      # Detection confidence threshold
cluster_tolerance: 0.2         # Euclidean clustering tolerance
min_cluster_size: 30          # Minimum points per cluster
max_cluster_size: 10000       # Maximum points per cluster
min_height: 0.5               # Minimum human height (m)
max_height: 2.0               # Maximum human height (m)
person_width: 0.5             # Expected person width (m)
```

#### Robust Detection
```yaml
max_detection_range: 15.0     # Maximum detection distance (m)
min_detection_range: 0.5      # Minimum detection distance (m)
ground_tolerance: 0.15        # Ground plane tolerance (m)
outlier_threshold: 1.0        # Statistical outlier removal
min_point_density: 6.0        # Minimum points per cubic meter
aspect_ratio_threshold: 3.0   # Maximum height/width ratio
symmetry_threshold: 0.2       # Minimum symmetry score
```

#### Standing Human Detection
```yaml
voxel_leaf_size: 0.08         # Voxel grid size (larger = more points)
adaptive_clustering: true     # Enable adaptive clustering
standing_density_factor: 0.7  # Density factor for standing humans
min_stationary_points: 20     # Minimum points for standing detection
```

### Tracker Parameters (`config/tracker_params.yaml`)

#### Multi-Target Tracking
```yaml
max_distance: 1.0             # Maximum association distance (m)
max_age: 10                   # Maximum track age before deletion
min_hits: 3                   # Minimum hits to confirm track
process_rate: 10.0            # Processing rate (Hz)
```

#### Kalman Filter Effectiveness Parameters

**Process Noise Tuning**:
```yaml
kalman_process_noise_pos: 0.3      # Position process noise
kalman_process_noise_vel: 0.8      # Velocity process noise  
kalman_process_noise_acc: 1.5      # Acceleration process noise
```

**Measurement Noise Adaptation**:
```yaml
kalman_base_measurement_noise: 0.08    # Base measurement noise
kalman_max_measurement_noise: 0.8      # Maximum adaptive noise
kalman_innovation_threshold: 1.8       # Innovation threshold
```

**Advanced Settings**:
```yaml
kalman_initial_position_uncertainty: 50.0   # Initial position uncertainty
kalman_initial_velocity_uncertainty: 10.0   # Initial velocity uncertainty
kalman_initial_acceleration_uncertainty: 100.0  # Initial acceleration uncertainty
kalman_prediction_uncertainty_growth: 0.3   # Uncertainty growth rate
kalman_innovation_forgetting_factor: 0.85   # Innovation forgetting factor
kalman_motion_smoothing_factor: 0.95        # Motion smoothing factor
```

**Tracking Quality Enhancement**:
```yaml
kalman_enable_joseph_form: true             # Numerical stability
kalman_enable_outlier_rejection: true       # Outlier rejection
kalman_outlier_threshold: 3.0              # Outlier threshold
kalman_max_human_velocity: 3.0             # Max velocity (m/s)
kalman_max_human_acceleration: 5.0         # Max acceleration (m/s²)
kalman_enable_velocity_gating: true        # Velocity gating
```

## 📡 Topics and Services

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud input |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/human_detections` | `human_tracker_ros2/DetectedHuman` | Individual human detections |
| `/human_detections_pointcloud` | `sensor_msgs/PointCloud2` | Colored pointclouds of detected humans |
| `/tracked_humans` | `human_tracker_ros2/TrackedHumans` | Array of tracked humans |
| `/tracked_humans_markers` | `visualization_msgs/MarkerArray` | Visualization markers for RViz |

### Custom Messages

**DetectedHuman.msg**:
```
std_msgs/Header header
int32 detection_id
geometry_msgs/Point position
geometry_msgs/Vector3 size
float64 confidence
sensor_msgs/PointCloud2 pointcloud
```

**TrackedHuman.msg**:
```
std_msgs/Header header
int32 track_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
float64 confidence
int32 age
int32 hits
```

**TrackedHumans.msg**:
```
std_msgs/Header header
TrackedHuman[] humans
```

## 📊 Visualization

### RViz Configuration

1. **Set Fixed Frame**: `livox_frame`

2. **Add Displays**:
   - **Raw LiDAR**: `/livox/lidar` (PointCloud2)
   - **Human Detections**: `/human_detections_pointcloud` (PointCloud2)
   - **Tracking Markers**: `/tracked_humans_markers` (MarkerArray)

3. **Color Configuration**:
   - **Human pointclouds**: Unique intensity gradients per human
   - **Tracking arrows**: Velocity direction and magnitude
   - **Track IDs**: Text labels for identification

### Visualization Features

- **Individual Human Pointclouds**: Each detected human gets a unique color
- **Velocity Arrows**: Direction and speed visualization
- **Track ID Labels**: Persistent identification numbers
- **Tracking History**: Trajectory trails (optional)

## 🔧 Performance Tuning

### For Different Environments

**Indoor Environments**:
```yaml
max_detection_range: 8.0
min_point_density: 8.0
kalman_base_measurement_noise: 0.05
```

**Outdoor Open Areas**:
```yaml
max_detection_range: 20.0
min_point_density: 4.0
kalman_max_human_velocity: 5.0
```

**Crowded Areas**:
```yaml
cluster_tolerance: 0.15
min_cluster_size: 25
kalman_outlier_threshold: 2.5
```

### Standing Human Detection Optimization

**For Better Standing Detection**:
```yaml
standing_density_factor: 0.6
min_stationary_points: 15
voxel_leaf_size: 0.06
symmetry_threshold: 0.15
```

**For Moving Human Priority**:
```yaml
standing_density_factor: 0.8
min_stationary_points: 30
kalman_motion_smoothing_factor: 0.9
```

### Kalman Filter Effectiveness Tuning

**High Accuracy Mode**:
```yaml
kalman_process_noise_pos: 0.2
kalman_enable_joseph_form: true
kalman_enable_outlier_rejection: true
kalman_outlier_threshold: 2.5
```

**Fast Tracking Mode**:
```yaml
kalman_process_noise_vel: 1.2
kalman_motion_smoothing_factor: 0.85
kalman_max_human_acceleration: 8.0
```

**Stability Mode**:
```yaml
kalman_innovation_forgetting_factor: 0.9
kalman_prediction_uncertainty_growth: 0.2
kalman_enable_velocity_gating: true
```

## 🐛 Troubleshooting

### Common Issues

**No Detections**:
- Check LiDAR topic: `ros2 topic echo /livox/lidar`
- Verify parameter ranges: `min_cluster_size`, `max_detection_range`
- Check ground plane segmentation: `ground_tolerance`

**False Positives**:
- Increase `confidence_threshold`
- Tune `min_point_density` and `symmetry_threshold`
- Adjust `aspect_ratio_threshold`

**Standing Humans Not Detected**:
- Lower `standing_density_factor` to 0.6
- Reduce `min_stationary_points`
- Increase `voxel_leaf_size` to preserve more points

**Tracking Instability**:
- Tune Kalman filter parameters
- Increase `kalman_innovation_threshold`
- Enable `kalman_enable_outlier_rejection`

**Parameter Type Errors**:
- Ensure boolean parameters are properly formatted in YAML
- Check parameter names match exactly (case-sensitive)

### Debug Topics

Monitor these topics for debugging:
```bash
ros2 topic echo /human_detections
ros2 topic hz /livox/lidar
ros2 node info /human_detector
ros2 node info /human_tracker
```

### Log Analysis

Enable debug logging:
```bash
ros2 launch human_tracker_ros2 human_tracker.launch.py --log-level debug
```

## 🤝 Contributing

### Development Setup

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Commit changes**: `git commit -m 'Add amazing feature'`
4. **Push to branch**: `git push origin feature/amazing-feature`
5. **Open a Pull Request**

### Code Standards

- Follow ROS2 coding standards
- Add comprehensive documentation
- Include unit tests for new features
- Update README for new parameters

### Testing

Run the test suite:
```bash
colcon test --packages-select human_tracker_ros2
colcon test-result --verbose
```

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- **Saun Nguyen** - *Initial work and enhancements* - [son94227@gmail.com](mailto:son94227@gmail.com)

## 🙏 Acknowledgments

- ROS2 community for the excellent framework
- PCL developers for robust point cloud processing
- Contributors to the Eigen library
- LiDAR sensor manufacturers for reliable hardware

## 📚 References

1. [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
2. [Point Cloud Library (PCL)](https://pointclouds.org/)
3. [Kalman Filter Theory](https://en.wikipedia.org/wiki/Kalman_filter)
4. [Multi-Object Tracking Algorithms](https://arxiv.org/abs/1603.00831)

---

For more information, issues, or feature requests, please visit our [GitHub repository](https://github.com/your-repo/human_tracker_ros2).