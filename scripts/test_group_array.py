#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from human_tracker_ros2.msg import GroupStateArray

class GroupStateArraySubscriber(Node):
    def __init__(self):
        super().__init__('group_state_array_test')
        self.subscription = self.create_subscription(
            GroupStateArray,
            'group_state_ndarray',
            self.group_state_callback,
            10)
        self.get_logger().info('Group state ndarray subscriber started')

    def group_state_callback(self, msg):
        actual_groups = msg.num_groups
        self.get_logger().info(f'Received {actual_groups} actual groups (out of 15 slots)')
        
        # Always reshape to 15 groups (fixed size)
        data = np.array(msg.data.data).reshape(15, 5)
        
        # Show actual groups (non-zero)
        for i in range(actual_groups):
            x, y, vx, vy, r = data[i]
            self.get_logger().info(
                f'Group {i}: pos=({x:.2f}, {y:.2f}), '
                f'vel=({vx:.2f}, {vy:.2f}), radius={r:.2f}')
        
        # Show that remaining slots are zeros
        if actual_groups < 15:
            self.get_logger().info(f'Slots {actual_groups}-14 are filled with zeros')

def main(args=None):
    rclpy.init(args=args)
    node = GroupStateArraySubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()