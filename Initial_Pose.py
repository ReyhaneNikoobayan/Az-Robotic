#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat

def create_pose_stamped(navigator, x, y, yaw):
    # transforms3d returns (w, x, y, z)
    q_w, q_x, q_y, q_z = euler2quat(0.0, 0.0, yaw)

    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    return pose

def main():
    rclpy.init()

    nav = BasicNavigator()

    # Wait until Nav2 is active (IMPORTANT)
    nav.waitUntilNav2Active()

    # Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0,0.0)
    nav.setInitialPose(initial_pose)

    print("Initial pose published!")

    print(nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
