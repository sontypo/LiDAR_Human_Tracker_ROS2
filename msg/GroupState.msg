std_msgs/Header header
int32 group_id
geometry_msgs/Point position      # Group centroid position [x, y, z]
geometry_msgs/Vector3 velocity    # Group centroid velocity [vx, vy, vz]
float64 safety_radius            # Safety space radius of the group
int32 member_count               # Number of humans in this group
int32[] member_ids               # Track IDs of group members