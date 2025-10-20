#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

class TestPointCloudPublisher : public rclcpp::Node
{
public:
  TestPointCloudPublisher() : Node("test_pointcloud_publisher")
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TestPointCloudPublisher::publishTestCloud, this));
    
    RCLCPP_INFO(get_logger(), "Test pointcloud publisher started");
  }

private:
  void publishTestCloud()
  {
    // Create multiple human-like pointcloud clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Generate multiple cylindrical clusters that look like people
    std::vector<std::array<float, 3>> human_positions = {
      {2.0f, -1.0f, 0.0f},   // Human 1: 2m front, 1m left
      {3.0f, 0.5f, 0.0f},    // Human 2: 3m front, 0.5m right  
      {1.5f, 1.5f, 0.0f},    // Human 3: 1.5m front, 1.5m right
      {4.0f, -0.5f, 0.0f},   // Human 4: 4m front, 0.5m left
      {2.5f, 0.0f, 0.0f}     // Human 5: 2.5m front, centered
    };
    
    std::vector<float> human_heights = {1.8f, 1.6f, 1.4f, 1.75f, 1.65f};  // Different heights
    
    // Generate each human cluster
    for (size_t i = 0; i < human_positions.size(); ++i) {
      float center_x = human_positions[i][0];
      float center_y = human_positions[i][1];
      float base_z = human_positions[i][2];
      float height = human_heights[i];
      float radius = 0.25f + (i * 0.05f);  // Slightly different widths
      
      for (float z = base_z; z < base_z + height; z += 0.08f) {
        for (float angle = 0; angle < 2 * M_PI; angle += 0.3f) {
          for (float r = 0; r < radius; r += 0.06f) {
            pcl::PointXYZ point;
            point.x = center_x + r * cos(angle);
            point.y = center_y + r * sin(angle);
            point.z = z;
            cloud->push_back(point);
          }
        }
      }
    }
    
    // Add some ground points around all people
    for (int i = -15; i < 15; i++) {
      for (int j = -15; j < 15; j++) {
        // Skip areas around people
        bool skip = false;
        for (const auto& pos : human_positions) {
          if (abs(i * 0.1f - pos[1]) < 1.0f && abs(j * 0.1f - (pos[0] - 1.0f)) < 1.0f) {
            skip = true;
            break;
          }
        }
        if (!skip) {
          pcl::PointXYZ point;
          point.x = 1.0f + j * 0.1f;
          point.y = i * 0.1f;
          point.z = -0.1f;  // Slightly below ground
          cloud->push_back(point);
        }
      }
    }
    
    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "livox_frame";
    cloud_msg.header.stamp = now();
    
    publisher_->publish(cloud_msg);
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
                        "Published test cloud with %zu points representing %zu humans", 
                        cloud->size(), human_positions.size());
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestPointCloudPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}