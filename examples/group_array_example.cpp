/*
 * Example C++ code for processing fixed 15-group array format
 */

#include <rclcpp/rclcpp.hpp>
#include "human_tracker_ros2/msg/group_state_array.hpp"

class GroupArrayProcessor : public rclcpp::Node
{
public:
  GroupArrayProcessor() : Node("group_array_processor")
  {
    subscription_ = this->create_subscription<human_tracker_ros2::msg::GroupStateArray>(
      "group_state_ndarray", 10,
      std::bind(&GroupArrayProcessor::groupArrayCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Group array processor started");
  }

private:
  void groupArrayCallback(const human_tracker_ros2::msg::GroupStateArray::SharedPtr msg)
  {
    int actual_groups = msg->num_groups;
    const auto& data = msg->data.data;
    
    RCLCPP_INFO(this->get_logger(), "Received %d actual groups (out of 15 slots)", actual_groups);
    
    // Process actual groups (data is always 15*5 = 75 elements)
    for (int i = 0; i < actual_groups; ++i) {
      double x = data[i*5 + 0];   // x position
      double y = data[i*5 + 1];   // y position  
      double vx = data[i*5 + 2];  // x velocity
      double vy = data[i*5 + 3];  // y velocity
      double r = data[i*5 + 4];   // safety radius
      
      RCLCPP_INFO(this->get_logger(), 
        "Group %d: pos=(%.2f, %.2f), vel=(%.2f, %.2f), radius=%.2f",
        i, x, y, vx, vy, r);
    }
    
    // Verify that remaining slots are zeros (optional check)
    for (int i = actual_groups; i < 15; ++i) {
      double sum = data[i*5 + 0] + data[i*5 + 1] + data[i*5 + 2] + data[i*5 + 3] + data[i*5 + 4];
      if (sum != 0.0) {
        RCLCPP_WARN(this->get_logger(), "Non-zero values found in slot %d", i);
      }
    }
  }
  
  rclcpp::Subscription<human_tracker_ros2::msg::GroupStateArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroupArrayProcessor>());
  rclcpp::shutdown();
  return 0;
}