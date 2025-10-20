# Message for a single tracked human
std_msgs/Header header

# Unique tracking ID that persists across frames
uint32 track_id

# Age of track (number of frames)
uint32 track_age

# Position in world frame
geometry_msgs/Pose pose

# Linear velocity in world frame
geometry_msgs/Vector3 velocity

# Covariance matrices for position and velocity
float64[9] position_covariance
float64[9] velocity_covariance

# Track status (0=new, 1=tracked, 2=occluded, 3=lost)
uint8 track_status

# Classification confidence
float32 confidence