import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from custom_interfaces.msg import SyncData
from collections import deque

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        self.pose_buffer = deque(maxlen=100)  # Store recent poses
        self.publisher_ = self.create_publisher(SyncData, '/synced_data', 10)
        self.create_subscription(PoseStamped, '/ee_pose_fast', self.ee_pose_callback, 10)
        self.create_subscription(Float32MultiArray, '/anyskin_measurements', self.measurements_callback, 10)
        self.get_logger().info('Data synchronizer node started')

    def ee_pose_callback(self, msg):
        # Store pose with its timestamp
        self.pose_buffer.append(msg)

    def measurements_callback(self, msg):
        # Find the closest pose in time (not newer than measurement)
        measurement_stamp = msg.header.stamp if hasattr(msg, 'header') else self.get_clock().now().to_msg()
        closest_pose = None
        min_diff = None
        for pose in self.pose_buffer:
            pose_stamp = pose.header.stamp
            # Only consider poses not newer than measurement
            if self._stamp_to_float(pose_stamp) <= self._stamp_to_float(measurement_stamp):
                diff = abs(self._stamp_to_float(measurement_stamp) - self._stamp_to_float(pose_stamp))
                if min_diff is None or diff < min_diff:
                    min_diff = diff
                    closest_pose = pose
        if closest_pose is not None:
            out_msg = SyncData()
            out_msg.stamp = measurement_stamp
            out_msg.ee_pose = closest_pose
            out_msg.measurements = msg
            self.publisher_.publish(out_msg)
            # self.get_logger().info('Published sensor-triggered synced data')
        else:
            self.get_logger().warn('No pose available for current measurement')

    def _stamp_to_float(self, stamp):
        # Convert ROS2 builtin_interfaces/Time to float seconds
        return stamp.sec + stamp.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()