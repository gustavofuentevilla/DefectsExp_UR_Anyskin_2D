#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("target_pose_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "/cartesian_compliance_controller/target_frame", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_target_pose)
        self.get_logger().info("Target Pose Publisher Node has been started.")

    def publish_target_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = (
            # Reference frame for the target end-effector pose,
            # can be changed as needed, 
            # another option could be "ur_base_link"
            "world"  
        )   
        pose.pose.position.x = 0.28
        pose.pose.position.y = 0.22
        pose.pose.position.z = 0.0
        # Identity quaternion (no rotation) [0,0,0,1]
        pose.pose.orientation.x = 0.7071
        pose.pose.orientation.y = -0.7071
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        self.publisher_.publish(pose)
        self.get_logger().info(
            f"Published target pose: position=({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
