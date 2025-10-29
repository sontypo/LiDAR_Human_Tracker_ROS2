#!/usr/bin/env python3
"""
Proof of Concept: Verify that /group_state_ndarray always contains the 15 closest groups
"""

import rclpy
from rclpy.node import Node
import numpy as np
from human_tracker_ros2.msg import GroupStates, GroupStateArray

class ClosestGroupsVerifier(Node):
    def __init__(self):
        super().__init__('closest_groups_verifier')
        
        # Subscribe to both topics
        self.group_states_sub = self.create_subscription(
            GroupStates, 'group_states', self.group_states_callback, 10)
        self.group_array_sub = self.create_subscription(
            GroupStateArray, 'group_state_ndarray', self.group_array_callback, 10)
        
        self.latest_all_groups = None
        self.latest_closest_15 = None
        
        self.get_logger().info('Closest groups verifier started')

    def group_states_callback(self, msg):
        """Store all detected groups for comparison"""
        self.latest_all_groups = []
        for group in msg.groups:
            # Calculate distance from reference point (0,0)
            dist = np.sqrt(group.position.x**2 + group.position.y**2)
            self.latest_all_groups.append({
                'id': group.group_id,
                'position': (group.position.x, group.position.y),
                'distance': dist
            })
        
        # Sort by distance to verify order
        self.latest_all_groups.sort(key=lambda g: g['distance'])
        self.verify_closest_groups()

    def group_array_callback(self, msg):
        """Store the 15 groups from ndarray for comparison"""
        actual_groups = msg.num_groups
        data = np.array(msg.data.data).reshape(15, 5)
        
        self.latest_closest_15 = []
        for i in range(actual_groups):
            x, y, vx, vy, r = data[i]
            dist = np.sqrt(x**2 + y**2)
            self.latest_closest_15.append({
                'index': i,
                'position': (x, y),
                'distance': dist
            })
        
        self.verify_closest_groups()

    def verify_closest_groups(self):
        """Verify that ndarray contains the 15 closest groups"""
        if self.latest_all_groups is None or self.latest_closest_15 is None:
            return
        
        total_groups = len(self.latest_all_groups)
        closest_15_count = len(self.latest_closest_15)
        
        self.get_logger().info(f'Verification: {total_groups} total groups, {closest_15_count} in ndarray')
        
        if total_groups <= 15:
            # If we have 15 or fewer groups, all should be included
            expected_count = total_groups
            if closest_15_count == expected_count:
                self.get_logger().info('✅ PASS: All groups included (≤15 total)')
            else:
                self.get_logger().error(f'❌ FAIL: Expected {expected_count}, got {closest_15_count}')
        else:
            # If we have more than 15 groups, verify the closest 15 are selected
            expected_15_closest = self.latest_all_groups[:15]  # First 15 after sorting by distance
            
            # Check distances
            expected_distances = [g['distance'] for g in expected_15_closest]
            actual_distances = [g['distance'] for g in self.latest_closest_15]
            
            max_expected_dist = max(expected_distances)
            max_actual_dist = max(actual_distances)
            
            self.get_logger().info(f'Expected max distance: {max_expected_dist:.2f}')
            self.get_logger().info(f'Actual max distance: {max_actual_dist:.2f}')
            
            # Verify that no group in ndarray is farther than the 15th closest
            if max_actual_dist <= max_expected_dist + 0.01:  # Small tolerance for floating point
                self.get_logger().info('✅ PASS: NDArray contains 15 closest groups')
                
                # Print the groups for verification
                self.get_logger().info('Closest 15 groups:')
                for i, group in enumerate(self.latest_closest_15):
                    self.get_logger().info(f'  {i}: pos=({group["position"][0]:.2f}, {group["position"][1]:.2f}), dist={group["distance"]:.2f}')
            else:
                self.get_logger().error('❌ FAIL: NDArray does not contain 15 closest groups')

def main(args=None):
    rclpy.init(args=args)
    node = ClosestGroupsVerifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()