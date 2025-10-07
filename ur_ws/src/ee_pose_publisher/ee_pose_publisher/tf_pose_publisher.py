#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class TfPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_pose_publisher')

        # Declare and get the update_rate parameter (default to 1 if not set)
        self.declare_parameter('update_rate', 1)
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value

        # Calculate timer period from update_rate (Hz)
        timer_period = 1.0 / update_rate if update_rate > 0 else 1.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, '/ee_pose_tf', 10)
        self.timer = self.create_timer(timer_period, self.publish_pose)
        self.parent_frame = 'world'
        self.child_frame = 'ee_link'

    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time())
            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            self.publisher.publish(pose)
        except Exception as e:
            self.get_logger().warn(f"Transform not available yet: {e}")
            # Do not raise or exit; just wait for the next timer tick

def main(args=None):
    rclpy.init(args=args)
    node = TfPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()