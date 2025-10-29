std_msgs/Header header
std_msgs/Float64MultiArray data  # Fixed 15 groups, each as [x, y, vx, vy, r]
int32 num_groups                 # Number of actual groups (non-zero entries)