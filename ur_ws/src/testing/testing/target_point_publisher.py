#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import factorial
import time


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("target_pose_publisher")
        self.target_pose_pub = self.create_publisher(PoseStamped, '/cartesian_compliance_controller/target_frame', 10)
        self.get_logger().info("Target Pose Publisher Node has been started.")
        self.last_pose = None
        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            '/ee_pose_fast',
            self.ee_pose_callback,
            10
        )
        self.desiredOrientation = np.array([
            -0.7071,
            0.7071,
            0.0000,
            0.0000
        ])
        self.desiredQuat = self.desiredOrientation.copy()
        self.last_Orientation = None

    def ee_pose_callback(self, msg):
        """
        Update last_synced_pose from direct PoseStamped messages on /ee_pose_fast.
        This provides a continuous pose feed even when /synced_data is updated only on measurements.
        """
        pose = msg.pose

        self.last_pose = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])

        self.last_Orientation = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        
        if np.dot(self.last_Orientation, self.desiredOrientation) < 0:
            self.desiredQuat = -self.desiredOrientation

    def rest_to_rest_trajectory(self, P_i, P_f, t_i, mvr_time, t, n):
        """
        Calcula una trayectoria rest-to-rest entre dos poses (P_i y P_f).

        Parámetros:
        -----------
        P_i : array_like
            Pose inicial (vector).
        P_f : array_like
            Pose final (vector).
        t_i : float
            Tiempo inicial del movimiento.
        mvr_time : float
            Duración total del movimiento.
        t : float
            Tiempo actual.
        n : int
            Número de derivadas igualadas a cero en los extremos.

        Retorna:
        --------
        X : ndarray
            Pose interpolada en el tiempo t.
        """

        if n < 1:
            raise ValueError("El orden de derivadas debe ser mayor o igual a 1")

        P_i = np.array(P_i, dtype=float)
        P_f = np.array(P_f, dtype=float)
        P_dim = len(P_i)

        t_f = t_i + mvr_time  # tiempo final

        if t < t_i:
            return P_i
        elif t > t_f:
            return P_f
        else:
            A = np.zeros((n+1, n+1, P_dim))
            Alpha = np.zeros((n+1, 1, P_dim))
            Lambda = np.zeros((n+1, 1, P_dim))
            Lambda[0, 0, :] = 1  

            f = np.zeros(P_dim)

            for r in range(P_dim):
                # Construcción de la matriz A
                for i in range(1, n+2):
                    for j in range(1, n+2):
                        A[i-1, j-1, r] = ((-1)**(j-1)) * factorial(n + j) / (
                            ((t_f - t_i)**(i-1)) * factorial(n + j - i + 1)
                        )

                # Resolución del sistema lineal
                Alpha[:, 0, r] = np.linalg.solve(A[:, :, r], Lambda[:, 0, r])

                # Cálculo del polinomio f(r)
                tau = (t - t_i) / (t_f - t_i)
                for j in range(0, n+1):
                    f[r] += ((-1)**j) * Alpha[j, 0, r] * tau**(n+1+j)

            X = P_i + (P_f - P_i) * f
            return X

    def publish_target_pose(self):

        # Esperar a tener una lectura válida de /ee_pose_fast
        while rclpy.ok() and self.last_pose is None:
            self.get_logger().info('Esperando datos de /ee_pose_fast...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        P_i = self.last_pose  # [x, y, z, qx, qy, qz, qw]
        # Pose objetivo del robot
        P_f = np.array([0.033, 0.140, 0.0, *self.desiredQuat])
        now = self.get_clock().now().seconds_nanoseconds()
        # Tiempo actual
        t_i = now[0] + now[1] * 1e-9
        # Duración de la maniobra en segundos
        mvr_time = 10.0  
        # Derivadas a cero en los extremos de trayectoria
        n_deriv = 2
        self.get_logger().info('Iniciando rutina de movimiento suave')
        t_f = t_i + mvr_time
        hold_time = 5.0
        period = 0.1  # seconds
        next_time = time.time() + period
        while rclpy.ok():
            # process incoming messages so subscriber callbacks update last_pose
            rclpy.spin_once(self, timeout_sec=0.0)
            now = self.get_clock().now().seconds_nanoseconds()
            t_now = now[0] + now[1] * 1e-9
            if t_now > t_f + hold_time:  
                break
            X = self.rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t_now, n_deriv)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(X[0])
            pose_msg.pose.position.y = float(X[1])
            pose_msg.pose.position.z = float(X[2])
            pose_msg.pose.orientation.x = float(X[3])
            pose_msg.pose.orientation.y = float(X[4])
            pose_msg.pose.orientation.z = float(X[5])
            pose_msg.pose.orientation.w = float(X[6])
            # publish target wrench (can be zero or customized)
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period

        self.get_logger().info('Rutina finalizada, pose objetivo alcanzada.')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    node.publish_target_pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
