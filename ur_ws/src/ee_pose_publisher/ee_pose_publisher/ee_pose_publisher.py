import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation

def pose_to_matrix(pose):
    trans = np.array([pose.position.x, pose.position.y, pose.position.z])
    rot = Rotation.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]).as_matrix()
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = trans
    return mat

def matrix_to_pose(mat):
    pose = PoseStamped()
    pose.pose.position.x = mat[0, 3]
    pose.pose.position.y = mat[1, 3]
    pose.pose.position.z = mat[2, 3]
    rot = Rotation.from_matrix(mat[:3, :3]).as_quat()
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]
    return pose

class EePosePublisher(Node):
    def __init__(self):
        super().__init__('ee_pose_publisher')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/ee_pose_fast', 10)

        # Static transform: ur5e_base in world
        self.T_world_ur5e_base = np.eye(4)
        # Example: translation (x, y, z) and quaternion (x, y, z, w)
        self.T_world_ur5e_base[:3, 3] = [0.460, 0.436, 0.910]
        self.T_world_ur5e_base[:3, :3] = Rotation.from_quat([0.0, 1.0, 0.0, 0.0]).as_matrix()

        # Static transform: ee_link in ur5e_tool0_controller
        self.T_tool0_controller_ee_link = np.eye(4)
        self.T_tool0_controller_ee_link[:3, 3] = [0.0, 0.0, -0.050]
        self.T_tool0_controller_ee_link[:3, :3] = Rotation.from_quat([0.0, 0.0, 0.0, 1.0]).as_matrix()

    def listener_callback(self, msg):
        # Pose of ur5e_tool0_controller in ur5e_base
        T_ur5e_base_tool0_controller = pose_to_matrix(msg.pose)

        # Compose transforms: world->ur5e_base->tool0_controller->ee_link
        T_world_ee_link = self.T_world_ur5e_base @ T_ur5e_base_tool0_controller @ self.T_tool0_controller_ee_link

        pose_msg = matrix_to_pose(T_world_ee_link)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()