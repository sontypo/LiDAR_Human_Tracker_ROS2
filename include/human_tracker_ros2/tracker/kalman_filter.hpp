#ifndef HUMAN_TRACKER_ROS2_KALMAN_FILTER_HPP
#define HUMAN_TRACKER_ROS2_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <map>
#include <string>

namespace human_tracker_ros2
{

class KalmanFilter
{
public:
  // Enhanced state vector: [x, y, z, vx, vy, vz, ax, ay, az] - added acceleration
  using StateVector = Eigen::Matrix<double, 9, 1>;
  using StateMatrix = Eigen::Matrix<double, 9, 9>;
  using MeasurementVector = Eigen::Matrix<double, 3, 1>;
  using MeasurementMatrix = Eigen::Matrix<double, 3, 9>;
  using MeasurementCovariance = Eigen::Matrix<double, 3, 3>;
  
  KalmanFilter();
  
  // Initialize filter with first measurement
  void init(const MeasurementVector& z, double timestamp);
  
  // Predict state at given timestamp
  StateVector predict(double timestamp);
  
  // Update state with new measurement
  void update(const MeasurementVector& z);
  
  // Get current state and covariance
  const StateVector& getState() const { return x_; }
  const StateMatrix& getCovariance() const { return P_; }
  
  // Get estimated velocity
  Eigen::Vector3d getVelocity() const { 
    return Eigen::Vector3d(x_(3), x_(4), x_(5)); 
  }
  
  // Get estimated acceleration
  Eigen::Vector3d getAcceleration() const {
    return Eigen::Vector3d(x_(6), x_(7), x_(8));
  }
  
  // Get position covariance (3x3 submatrix)
  Eigen::Matrix3d getPositionCovariance() const {
    return P_.block<3, 3>(0, 0);
  }
  
  // Get velocity covariance (3x3 submatrix)
  Eigen::Matrix3d getVelocityCovariance() const {
    return P_.block<3, 3>(3, 3);
  }
  
  // Get acceleration covariance (3x3 submatrix)
  Eigen::Matrix3d getAccelerationCovariance() const {
    return P_.block<3, 3>(6, 6);
  }
  
  // Adaptive noise adjustment based on innovation
  void adaptNoiseParameters(const MeasurementVector& innovation);
  
  // Enhanced parameter configuration
  void setParameters(const std::map<std::string, double>& params);
  
  // Enhanced update methods for increased effectiveness
  void updateWithOutlierRejection(const MeasurementVector& z);
  bool validateMeasurement(const MeasurementVector& z) const;
  
  // Advanced quality and validation metrics
  double getTrackingQuality() const;
  double getMahalanobisDistance(const MeasurementVector& z) const;
  void updateTrackingQuality();
  
  // Motion constraint and stability features
  void resetUncertainty();
  void applyMotionConstraints();
  void enableJosephForm(bool enable) { use_joseph_form_ = enable; }
  void enableOutlierRejection(bool enable) { enable_outlier_rejection_ = enable; }
  
  // Get motion uncertainty score (higher = more uncertain)
  double getMotionUncertainty() const;

private:
  // State vector and covariance
  StateVector x_;  // [x, y, z, vx, vy, vz, ax, ay, az]
  StateMatrix P_;  // State covariance
  
  // System matrices
  StateMatrix F_;  // State transition matrix
  StateMatrix Q_;  // Process noise covariance
  MeasurementMatrix H_;  // Measurement matrix
  MeasurementCovariance R_;  // Measurement noise covariance
  
  // Enhanced noise parameters
  double process_noise_pos_;
  double process_noise_vel_;
  double process_noise_acc_;
  
  // Adaptive parameters
  double base_measurement_noise_;
  double max_measurement_noise_;
  double innovation_threshold_;
  
  // Last timestamp for time delta calculation
  double last_timestamp_;
  int consecutive_predictions_;  // Track prediction-only cycles
  
  // Enhanced effectiveness parameters
  double initial_pos_uncertainty_;
  double initial_vel_uncertainty_;
  double initial_acc_uncertainty_;
  double prediction_uncertainty_growth_;
  double innovation_forgetting_factor_;
  double motion_smoothing_factor_;
  double max_human_velocity_;
  double max_human_acceleration_;
  double outlier_threshold_;
  
  // Advanced feature flags
  bool use_joseph_form_;
  bool enable_outlier_rejection_;
  bool enable_velocity_gating_;
  
  // Quality tracking
  double tracking_quality_score_;
  int successful_updates_;
  int total_predictions_;
};

} // namespace human_tracker_ros2

#endif // HUMAN_TRACKER_ROS2_KALMAN_FILTER_HPP