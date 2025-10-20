#include "human_tracker_ros2/tracker/kalman_filter.hpp"

namespace human_tracker_ros2
{

KalmanFilter::KalmanFilter()
  : process_noise_pos_(0.3),
    process_noise_vel_(0.8),
    process_noise_acc_(1.5),
    base_measurement_noise_(0.08),
    max_measurement_noise_(0.8),
    innovation_threshold_(1.8),
    last_timestamp_(0.0),
    consecutive_predictions_(0),
    initial_pos_uncertainty_(50.0),
    initial_vel_uncertainty_(10.0),
    initial_acc_uncertainty_(100.0),
    prediction_uncertainty_growth_(0.3),
    innovation_forgetting_factor_(0.85),
    motion_smoothing_factor_(0.95),
    max_human_velocity_(3.0),
    max_human_acceleration_(5.0),
    outlier_threshold_(3.0),
    use_joseph_form_(true),
    enable_outlier_rejection_(true),
    enable_velocity_gating_(true),
    tracking_quality_score_(1.0),
    successful_updates_(0),
    total_predictions_(0)
{
  // Initialize state and covariance with enhanced parameters
  x_.setZero();
  P_.setIdentity();
  P_.block<3, 3>(0, 0) *= initial_pos_uncertainty_;     // Position uncertainty
  P_.block<3, 3>(3, 3) *= initial_vel_uncertainty_;     // Velocity uncertainty  
  P_.block<3, 3>(6, 6) *= initial_acc_uncertainty_;     // Acceleration uncertainty
  
  // Measurement matrix (we only measure position)
  H_.setZero();
  H_.block<3, 3>(0, 0).setIdentity();
  
  // Initial measurement noise (adaptive)
  R_.setIdentity() * base_measurement_noise_;
  
  // Initialize quality tracking
  tracking_quality_score_ = 1.0;
  successful_updates_ = 0;
  total_predictions_ = 0;
}

void KalmanFilter::init(const MeasurementVector& z, double timestamp)
{
  x_.setZero();
  x_.segment<3>(0) = z;  // Initial position from measurement
  
  // Reset adaptive parameters
  consecutive_predictions_ = 0;
  R_.setIdentity() * base_measurement_noise_;
  
  last_timestamp_ = timestamp;
}

KalmanFilter::StateVector KalmanFilter::predict(double timestamp)
{
  // Calculate time delta
  double dt = timestamp - last_timestamp_;
  if (dt <= 0) {
    return x_;  // No prediction needed
  }
  
  // Enhanced state transition matrix for constant acceleration model
  F_.setIdentity();
  // Position updates: x = x + vx*dt + 0.5*ax*dt²
  F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * 0.5 * dt * dt;
  // Velocity updates: vx = vx + ax*dt
  F_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
  // Acceleration remains constant: ax = ax
  
  // Enhanced process noise covariance matrix
  Q_.setZero();
  
  // Position noise (influenced by velocity and acceleration uncertainty)
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  
  // Position-position block
  Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 
    (process_noise_pos_ * dt4 / 4.0 + process_noise_vel_ * dt3 / 3.0 + process_noise_acc_ * dt2 / 20.0);
  
  // Position-velocity cross terms
  Q_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * 
    (process_noise_vel_ * dt3 / 2.0 + process_noise_acc_ * dt2 / 8.0);
  Q_.block<3, 3>(3, 0) = Q_.block<3, 3>(0, 3);
  
  // Position-acceleration cross terms
  Q_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * (process_noise_acc_ * dt3 / 6.0);
  Q_.block<3, 3>(6, 0) = Q_.block<3, 3>(0, 6);
  
  // Velocity-velocity block
  Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 
    (process_noise_vel_ * dt2 + process_noise_acc_ * dt / 3.0);
  
  // Velocity-acceleration cross terms
  Q_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * (process_noise_acc_ * dt2 / 2.0);
  Q_.block<3, 3>(6, 3) = Q_.block<3, 3>(3, 6);
  
  // Acceleration-acceleration block
  Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (process_noise_acc_ * dt);
  
  // Increase uncertainty for consecutive predictions without measurements
  if (consecutive_predictions_ > 0) {
    double uncertainty_factor = 1.0 + consecutive_predictions_ * prediction_uncertainty_growth_;
    Q_ *= uncertainty_factor;
  }
  
  // Predict step
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  
  // Apply motion constraints for human-like motion
  applyMotionConstraints();
  
  // Update timestamp and prediction counter
  last_timestamp_ = timestamp;
  consecutive_predictions_++;
  total_predictions_++;
  
  return x_;
}

void KalmanFilter::update(const MeasurementVector& z)
{
  // Use enhanced update with outlier rejection if enabled
  if (enable_outlier_rejection_) {
    updateWithOutlierRejection(z);
  } else {
    // Standard update
    // Innovation
    MeasurementVector y = z - H_ * x_;
    
    // Adaptive noise adjustment based on innovation
    adaptNoiseParameters(y);
    
    // Innovation covariance
    MeasurementCovariance S = H_ * P_ * H_.transpose() + R_;
    
    // Kalman gain
    Eigen::Matrix<double, 9, 3> K = P_ * H_.transpose() * S.inverse();
    
    // Update state and covariance
    x_ = x_ + K * y;
    
    if (use_joseph_form_) {
      // Joseph form for numerical stability
      StateMatrix I_KH = StateMatrix::Identity() - K * H_;
      P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
    } else {
      P_ = (StateMatrix::Identity() - K * H_) * P_;
    }
    
    // Apply motion smoothing
    if (motion_smoothing_factor_ < 1.0) {
      Eigen::Vector3d prev_velocity = getVelocity();
      Eigen::Vector3d current_velocity = getVelocity();
      
      // Smooth velocity changes
      x_.segment<3>(3) = motion_smoothing_factor_ * prev_velocity + 
                        (1.0 - motion_smoothing_factor_) * current_velocity;
    }
  }
  
  // Reset consecutive predictions counter and update quality
  consecutive_predictions_ = 0;
  successful_updates_++;
  
  // Update tracking quality score
  updateTrackingQuality();
}

void KalmanFilter::adaptNoiseParameters(const MeasurementVector& innovation)
{
  // Calculate normalized innovation magnitude
  double innovation_magnitude = innovation.norm();
  double expected_innovation = std::sqrt(R_.trace() / 3.0);  // Average measurement noise
  
  // Enhanced adaptive noise adjustment
  if (innovation_magnitude > innovation_threshold_ * expected_innovation) {
    // Large innovation - reduce trust in measurement
    double scaling_factor = std::min(4.0, innovation_magnitude / expected_innovation);
    double new_noise = std::min(max_measurement_noise_, 
                               base_measurement_noise_ * scaling_factor);
    R_.setIdentity() * new_noise;
  } else {
    // Gradually return to base noise level with configurable forgetting factor
    double current_noise = R_(0, 0);
    double new_noise = innovation_forgetting_factor_ * current_noise + 
                      (1.0 - innovation_forgetting_factor_) * base_measurement_noise_;
    R_.setIdentity() * new_noise;
  }
}

double KalmanFilter::getMotionUncertainty() const
{
  // Combine position and velocity uncertainties
  double pos_uncertainty = P_.block<3, 3>(0, 0).trace();
  double vel_uncertainty = P_.block<3, 3>(3, 3).trace();
  double acc_uncertainty = P_.block<3, 3>(6, 6).trace();
  
  return pos_uncertainty + 0.1 * vel_uncertainty + 0.01 * acc_uncertainty;
}

void KalmanFilter::setParameters(const std::map<std::string, double>& params)
{
  // Update all configurable parameters for enhanced effectiveness
  auto it = params.find("kalman_process_noise_pos");
  if (it != params.end()) process_noise_pos_ = it->second;
  
  it = params.find("kalman_process_noise_vel");
  if (it != params.end()) process_noise_vel_ = it->second;
  
  it = params.find("kalman_process_noise_acc");
  if (it != params.end()) process_noise_acc_ = it->second;
  
  it = params.find("kalman_base_measurement_noise");
  if (it != params.end()) base_measurement_noise_ = it->second;
  
  it = params.find("kalman_max_measurement_noise");
  if (it != params.end()) max_measurement_noise_ = it->second;
  
  it = params.find("kalman_innovation_threshold");
  if (it != params.end()) innovation_threshold_ = it->second;
  
  it = params.find("kalman_initial_position_uncertainty");
  if (it != params.end()) initial_pos_uncertainty_ = it->second;
  
  it = params.find("kalman_initial_velocity_uncertainty");
  if (it != params.end()) initial_vel_uncertainty_ = it->second;
  
  it = params.find("kalman_initial_acceleration_uncertainty");
  if (it != params.end()) initial_acc_uncertainty_ = it->second;
  
  it = params.find("kalman_prediction_uncertainty_growth");
  if (it != params.end()) prediction_uncertainty_growth_ = it->second;
  
  it = params.find("kalman_innovation_forgetting_factor");
  if (it != params.end()) innovation_forgetting_factor_ = it->second;
  
  it = params.find("kalman_motion_smoothing_factor");
  if (it != params.end()) motion_smoothing_factor_ = it->second;
  
  it = params.find("kalman_max_human_velocity");
  if (it != params.end()) max_human_velocity_ = it->second;
  
  it = params.find("kalman_max_human_acceleration");
  if (it != params.end()) max_human_acceleration_ = it->second;
  
  it = params.find("kalman_outlier_threshold");
  if (it != params.end()) outlier_threshold_ = it->second;
  
  it = params.find("kalman_enable_joseph_form");
  if (it != params.end()) use_joseph_form_ = (it->second > 0.5);
  
  it = params.find("kalman_enable_outlier_rejection");
  if (it != params.end()) enable_outlier_rejection_ = (it->second > 0.5);
  
  it = params.find("kalman_enable_velocity_gating");
  if (it != params.end()) enable_velocity_gating_ = (it->second > 0.5);
}

void KalmanFilter::updateWithOutlierRejection(const MeasurementVector& z)
{
  // Check if measurement is an outlier using Mahalanobis distance
  if (!validateMeasurement(z)) {
    // Reject outlier measurement - only do prediction step
    return;
  }
  
  // Standard update if measurement is valid
  MeasurementVector y = z - H_ * x_;
  adaptNoiseParameters(y);
  
  MeasurementCovariance S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix<double, 9, 3> K = P_ * H_.transpose() * S.inverse();
  
  x_ = x_ + K * y;
  
  if (use_joseph_form_) {
    StateMatrix I_KH = StateMatrix::Identity() - K * H_;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
  } else {
    P_ = (StateMatrix::Identity() - K * H_) * P_;
  }
}

bool KalmanFilter::validateMeasurement(const MeasurementVector& z) const
{
  double mahalanobis_dist = getMahalanobisDistance(z);
  return mahalanobis_dist < outlier_threshold_;
}

double KalmanFilter::getMahalanobisDistance(const MeasurementVector& z) const
{
  MeasurementVector innovation = z - H_ * x_;
  MeasurementCovariance S = H_ * P_ * H_.transpose() + R_;
  
  return std::sqrt(innovation.transpose() * S.inverse() * innovation);
}

double KalmanFilter::getTrackingQuality() const
{
  return tracking_quality_score_;
}

void KalmanFilter::updateTrackingQuality()
{
  // Calculate tracking quality based on successful updates and uncertainty
  double update_ratio = (total_predictions_ > 0) ? 
    static_cast<double>(successful_updates_) / total_predictions_ : 1.0;
  
  double uncertainty_penalty = std::min(1.0, getMotionUncertainty() / 100.0);
  
  tracking_quality_score_ = update_ratio * (1.0 - uncertainty_penalty);
}

void KalmanFilter::applyMotionConstraints()
{
  // Apply velocity constraints for human-like motion
  if (enable_velocity_gating_) {
    Eigen::Vector3d velocity = getVelocity();
    double speed = velocity.norm();
    
    if (speed > max_human_velocity_) {
      // Scale velocity to maximum human speed
      velocity = velocity.normalized() * max_human_velocity_;
      x_.segment<3>(3) = velocity;
    }
  }
  
  // Apply acceleration constraints
  Eigen::Vector3d acceleration = getAcceleration();
  double accel_magnitude = acceleration.norm();
  
  if (accel_magnitude > max_human_acceleration_) {
    // Scale acceleration to maximum human acceleration
    acceleration = acceleration.normalized() * max_human_acceleration_;
    x_.segment<3>(6) = acceleration;
  }
}

void KalmanFilter::resetUncertainty()
{
  P_.setIdentity();
  P_.block<3, 3>(0, 0) *= initial_pos_uncertainty_;
  P_.block<3, 3>(3, 3) *= initial_vel_uncertainty_;
  P_.block<3, 3>(6, 6) *= initial_acc_uncertainty_;
  
  tracking_quality_score_ = 1.0;
}

} // namespace human_tracker_ros2