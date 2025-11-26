#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')

        # Define QoS settings to match TurtleBot3 LiDAR
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Match LiDAR QoS
            depth=10
        )

        # Publisher to publish velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to receive LiDAR data with corrected QoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile
        )

        # Timer to periodically call a function
        self.timer = self.create_timer(0.1, self.timer_fun)

        # Store LiDAR data
        self.data_lidar = None

        # State variable to track if the robot is avoiding an obstacle
        self.avoiding_obstacle = False

    def listener_callback(self, data):
        # Callback to process LiDAR data
        self.data_lidar = data

    def timer_fun(self):
        if self.avoiding_obstacle or not self.data_lidar:
            self.get_logger().info("Waiting or no LiDAR data available.")
            return

        velocity_command = Twist()
        velocity_command.linear.x = 0.1
        self.publisher.publish(velocity_command)
        
        self.get_logger().info("Publishing forward velocity.")
        self.publisher.publish(velocity_command)

        # Analyze the LiDAR data: first 45 and last 45 readings
        first_45_ranges = self.data_lidar.ranges[:45]
        last_45_ranges = self.data_lidar.ranges[-45:]

        # Find the minimum distance in the selected ranges
        min_first_45 = min(first_45_ranges) if first_45_ranges else float('inf')
        min_last_45 = min(last_45_ranges) if last_45_ranges else float('inf')

        # Determine the closest obstacle
        min_distance = min(min_first_45, min_last_45)

        self.get_logger().info(f"Current min distance: {min_distance:.2f} meters")

        # Check if any of the ranges are below 0.7 meters
        if any(r < 0.7 for r in first_45_ranges) or any(r < 0.7 for r in last_45_ranges):
            self.get_logger().info(f"Obstacle detected! Stopping. Minimum distance: {min_distance:.2f}")
            velocity_command.linear.x = 0.0
            self.publisher.publish(velocity_command)

            # Start the obstacle avoidance sequence
            self.avoid_obstacle()
        else:
            self.get_logger().info("No obstacle detected. Moving forward.")
            self.publisher.publish(velocity_command)

    def avoid_obstacle(self):
        self.avoiding_obstacle = True

        # Stop and wait for 2 seconds
        self.get_logger().info("Stopping and waiting for 2 seconds...")
        time.sleep(2)  # Pause for 2 seconds

        # Rotate 90 degrees clockwise
        self.get_logger().info("Rotating 90 degrees clockwise...")
        velocity_command = Twist()
        velocity_command.angular.z = -0.5  # Rotate clockwise at 0.5 rad/s
        rotation_time = 45 / (0.5 * 57.3)  # Time required to rotate 90 degrees
        start_time = self.get_clock().now().to_msg().sec

        while (self.get_clock().now().to_msg().sec - start_time) < rotation_time:
            self.publisher.publish(velocity_command)
            time.sleep(0.1)  # Sleep to avoid over-publishing

        # Stop rotation
        velocity_command.angular.z = 0.0
        self.publisher.publish(velocity_command)

        # Wait for another 2 seconds after turning
        self.get_logger().info("Waiting for 2 seconds after rotation...")
        time.sleep(2)  # Pause for 2 seconds

        # Check if any obstacle is nearby after the wait
        self.get_logger().info("Checking for obstacles before resuming...")
        if self.data_lidar:
            first_45_ranges = self.data_lidar.ranges[:45]
            last_45_ranges = self.data_lidar.ranges[-45:]
            if any(r < 0.7 for r in first_45_ranges) or any(r < 0.7 for r in last_45_ranges):
                self.get_logger().info("Obstacle still nearby. Staying stopped.")
            else:
                self.get_logger().info("No nearby obstacles. Resuming forward motion.")
                velocity_command.linear.x = 0.5  # Move forward
                self.publisher.publish(velocity_command)

        self.avoiding_obstacle = False

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

