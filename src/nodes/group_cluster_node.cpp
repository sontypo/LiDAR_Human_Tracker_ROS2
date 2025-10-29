#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include "human_tracker_ros2/msg/tracked_humans.hpp"
#include "human_tracker_ros2/msg/group_states.hpp"
#include "human_tracker_ros2/msg/group_state_array.hpp"
#include "human_tracker_ros2/clustering/group_cluster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace human_tracker_ros2
{

class GroupClusterNode : public rclcpp::Node
{
public:
  GroupClusterNode() : Node("group_cluster")
  {
    // Declare parameters
    declareParameters();
    
    // Initialize group clustering with parameters
    GroupClusterParams params = loadParameters();
    group_cluster_ = std::make_shared<GroupCluster>(params);
    
    // Setup publishers
    group_states_pub_ = create_publisher<msg::GroupStates>("group_states", 10);
    group_state_array_pub_ = create_publisher<msg::GroupStateArray>("group_state_ndarray", 10);
    group_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("group_markers", 10);
    
    // Setup subscriber
    tracked_humans_sub_ = create_subscription<msg::TrackedHumans>(
      "tracked_humans", 10,
      std::bind(&GroupClusterNode::trackedHumansCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Group cluster node initialized");
    RCLCPP_INFO(get_logger(), "Max groups: %d, Max group distance: %.2f m", 
                params.max_groups, params.max_group_distance);
  }

private:
  void declareParameters()
  {
    declare_parameter("max_group_distance", 2.0);
    declare_parameter("min_safety_radius", 0.8);
    declare_parameter("max_safety_radius", 3.0);
    declare_parameter("safety_radius_factor", 1.5);
    declare_parameter("max_groups", 15);
    declare_parameter("group_velocity_threshold", 0.1);
    declare_parameter("treat_individual_as_group", true);
    declare_parameter("reference_x", 0.0);
    declare_parameter("reference_y", 0.0);
  }
  
  GroupClusterParams loadParameters()
  {
    GroupClusterParams params;
    params.max_group_distance = get_parameter("max_group_distance").as_double();
    params.min_safety_radius = get_parameter("min_safety_radius").as_double();
    params.max_safety_radius = get_parameter("max_safety_radius").as_double();
    params.safety_radius_factor = get_parameter("safety_radius_factor").as_double();
    params.max_groups = get_parameter("max_groups").as_int();
    params.group_velocity_threshold = get_parameter("group_velocity_threshold").as_double();
    params.treat_individual_as_group = get_parameter("treat_individual_as_group").as_bool();
    params.reference_x = get_parameter("reference_x").as_double();
    params.reference_y = get_parameter("reference_y").as_double();
    return params;
  }
  
  void trackedHumansCallback(const msg::TrackedHumans::SharedPtr msg)
  {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                         "Received %zu tracked humans for grouping", msg->humans.size());
    
    // Perform group clustering
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    msg::GroupStates group_states = group_cluster_->clusterHumans(*msg, timestamp);
    
    // Publish group states
    group_states_pub_->publish(group_states);
    
    // Publish group state ndarray
    publishGroupStateArray(group_states);
    
    // Publish visualization markers
    publishGroupMarkers(group_states);
    
    RCLCPP_DEBUG(get_logger(), "Published %d groups", group_states.total_groups);
  }
  
  void publishGroupStateArray(const msg::GroupStates& group_states)
  {
    msg::GroupStateArray array_msg;
    array_msg.header = group_states.header;
    
    // Add actual groups (up to 15)
    int actual_groups = std::min(static_cast<int>(group_states.groups.size()), 15);
    array_msg.num_groups = actual_groups;  // Number of actual (non-zero) groups
    
    // Convert each group to [x, y, vx, vy, r] format
    std::vector<double> data_vector;
    data_vector.reserve(15 * 5);  // Reserve space for 15 groups * 5 values each
    for (int i = 0; i < actual_groups; ++i) {
      const auto& group = group_states.groups[i];
      data_vector.push_back(group.position.x);     // x
      data_vector.push_back(group.position.y);     // y  
      data_vector.push_back(group.velocity.x);     // vx
      data_vector.push_back(group.velocity.y);     // vy
      data_vector.push_back(group.safety_radius);  // r
    }
    
    // Fill remaining slots with zeros [0.0, 0.0, 0.0, 0.0, 0.0]
    int remaining_slots = 15 - actual_groups;
    for (int i = 0; i < remaining_slots; ++i) {
      data_vector.push_back(0.0);  // x
      data_vector.push_back(0.0);  // y
      data_vector.push_back(0.0);  // vx
      data_vector.push_back(0.0);  // vy
      data_vector.push_back(0.0);  // r
    }
    
    // Set up the MultiArray structure (always 15 groups)
    array_msg.data.layout.dim.resize(2);
    array_msg.data.layout.dim[0].label = "groups";
    array_msg.data.layout.dim[0].size = 15;      // Fixed to 15 groups
    array_msg.data.layout.dim[0].stride = 15 * 5; // 15 groups * 5 values
    array_msg.data.layout.dim[1].label = "group_data";
    array_msg.data.layout.dim[1].size = 5;  // [x, y, vx, vy, r]
    array_msg.data.layout.dim[1].stride = 5;
    array_msg.data.layout.data_offset = 0;
    array_msg.data.data = data_vector;
    
    group_state_array_pub_->publish(array_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published fixed 15-group array with %d actual groups", actual_groups);
  }
  
  void publishGroupMarkers(const msg::GroupStates& group_states)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = group_states.header;
    clear_marker.ns = "group_clusters";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Create markers for each group
    for (size_t i = 0; i < group_states.groups.size(); ++i) {
      const auto& group = group_states.groups[i];
      
      // Group centroid marker
      visualization_msgs::msg::Marker centroid_marker;
      centroid_marker.header = group_states.header;
      centroid_marker.ns = "group_centroids";
      centroid_marker.id = group.group_id;
      centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
      centroid_marker.action = visualization_msgs::msg::Marker::ADD;
      centroid_marker.pose.position = group.position;
      centroid_marker.pose.orientation.w = 1.0;
      centroid_marker.scale.x = 0.3;
      centroid_marker.scale.y = 0.3;
      centroid_marker.scale.z = 0.3;
      
      // Color based on group size
      if (group.member_count == 1) {
        centroid_marker.color.r = 0.0; centroid_marker.color.g = 1.0; centroid_marker.color.b = 0.0; // Green for individual
      } else if (group.member_count <= 3) {
        centroid_marker.color.r = 1.0; centroid_marker.color.g = 1.0; centroid_marker.color.b = 0.0; // Yellow for small group
      } else {
        centroid_marker.color.r = 1.0; centroid_marker.color.g = 0.0; centroid_marker.color.b = 0.0; // Red for large group
      }
      centroid_marker.color.a = 0.8;
      marker_array.markers.push_back(centroid_marker);
      
      // Safety radius circle
      visualization_msgs::msg::Marker safety_circle;
      safety_circle.header = group_states.header;
      safety_circle.ns = "safety_circles";
      safety_circle.id = group.group_id;
      safety_circle.type = visualization_msgs::msg::Marker::CYLINDER;
      safety_circle.action = visualization_msgs::msg::Marker::ADD;
      safety_circle.pose.position = group.position;
      safety_circle.pose.position.z = 0.1; // Slightly above ground
      safety_circle.pose.orientation.w = 1.0;
      safety_circle.scale.x = group.safety_radius * 2.0; // Diameter
      safety_circle.scale.y = group.safety_radius * 2.0; // Diameter  
      safety_circle.scale.z = 0.05; // Thin cylinder
      safety_circle.color.r = 1.0;
      safety_circle.color.g = 0.5;
      safety_circle.color.b = 0.0;
      safety_circle.color.a = 0.3;
      marker_array.markers.push_back(safety_circle);
      
      // Group velocity arrow
      double velocity_magnitude = std::sqrt(
        group.velocity.x * group.velocity.x + 
        group.velocity.y * group.velocity.y);
      
      if (velocity_magnitude > 0.1) { // Only show if moving
        visualization_msgs::msg::Marker velocity_arrow;
        velocity_arrow.header = group_states.header;
        velocity_arrow.ns = "group_velocities";
        velocity_arrow.id = group.group_id;
        velocity_arrow.type = visualization_msgs::msg::Marker::ARROW;
        velocity_arrow.action = visualization_msgs::msg::Marker::ADD;
        
        // Arrow from centroid to velocity direction
        geometry_msgs::msg::Point start = group.position;
        geometry_msgs::msg::Point end = group.position;
        end.x += group.velocity.x;
        end.y += group.velocity.y;
        end.z += group.velocity.z;
        
        velocity_arrow.points.push_back(start);
        velocity_arrow.points.push_back(end);
        
        velocity_arrow.scale.x = 0.1; // Shaft diameter
        velocity_arrow.scale.y = 0.2; // Head diameter
        velocity_arrow.scale.z = 0.3; // Head length
        velocity_arrow.color.r = 0.0;
        velocity_arrow.color.g = 0.0;
        velocity_arrow.color.b = 1.0;
        velocity_arrow.color.a = 0.8;
        marker_array.markers.push_back(velocity_arrow);
      }
      
      // Group ID text
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = group_states.header;
      text_marker.ns = "group_ids";
      text_marker.id = group.group_id;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = group.position;
      text_marker.pose.position.z += 0.5; // Above centroid
      text_marker.pose.orientation.w = 1.0;
      text_marker.text = "G" + std::to_string(group.group_id) + "(" + std::to_string(group.member_count) + ")";
      text_marker.scale.z = 0.3; // Text size
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      marker_array.markers.push_back(text_marker);
    }
    
    group_markers_pub_->publish(marker_array);
  }
  
  // Members
  std::shared_ptr<GroupCluster> group_cluster_;
  rclcpp::Publisher<msg::GroupStates>::SharedPtr group_states_pub_;
  rclcpp::Publisher<msg::GroupStateArray>::SharedPtr group_state_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr group_markers_pub_;
  rclcpp::Subscription<msg::TrackedHumans>::SharedPtr tracked_humans_sub_;
};

} // namespace human_tracker_ros2

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<human_tracker_ros2::GroupClusterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}