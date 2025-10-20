# Message for a single detected human
std_msgs/Header header

# Unique ID for this detection
uint32 detection_id

# Detection confidence score (0.0-1.0)
float32 confidence

# 3D position in sensor frame
geometry_msgs/Point position

# Bounding box in 3D (if available)
geometry_msgs/Vector3 dimensions

# Optional: Associated 2D image detection
sensor_msgs/RegionOfInterest image_roi

# Source of detection (lidar, camera, fusion)
string detection_source