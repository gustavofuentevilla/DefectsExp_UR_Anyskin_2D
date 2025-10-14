import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_interfaces.msg import SyncData
import numpy as np
from math import factorial
import time

# Commentarios:
# Posibles parámetros a ajustar desde el launch file:
# - Duración de las maniobras (mvr_time)
# - Frecuencia de publicación (period)
# - hold_time en cada posición final

class URMotionRoutinesNode(Node):
    def __init__(self):
        super().__init__('ur_motion_routines_node')
        self.get_logger().info('Nodo de rutinas de movimiento iniciado')
        # Publisher para el controlador cartesiano
        self.target_pose_pub = self.create_publisher(PoseStamped, '/cartesian_compliance_controller/target_frame', 10)
        # Subscriber a /synced_data
        self.last_synced_pose = None
        self.synced_data_sub = self.create_subscription(
            SyncData,
            '/synced_data',
            self.synced_data_callback,
            10
        )
        self.desiredOrientation = np.array([
            -0.7071,
            0.7071,
            0.0022,
            0.0056
        ])
        self.desiredQuat = self.desiredOrientation.copy()
        self.current_Orientation = None

    def synced_data_callback(self, msg):
        # Guarda la última pose cartesiana recibida
        pose = msg.ee_pose.pose
        # [x, y, z, qx, qy, qz, qw]
        self.last_synced_pose = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        
        self.current_Orientation = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        
        if np.dot(self.current_Orientation, self.desiredOrientation) < 0:
            self.desiredQuat = -self.desiredOrientation
            
    def rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t, n):
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
                    f[r] += ((-1)*j) * Alpha[j, 0, r] * tau*(n+1+j)

            X = P_i + (P_f - P_i) * f
            return X

    
    def move_to_initial_pose(self):
        """
        Mueve el robot suavemente a una posición inicial cartesiana usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /synced_data
        while rclpy.ok() and self.last_synced_pose is None:
            self.get_logger().info('Esperando datos de /synced_data para obtener la pose inicial...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        P_i = self.last_synced_pose  # [x, y, z, qx, qy, qz, qw]
        # Pose inicial deseada
        # P_f = np.array([0.03, 0.11, 0.1, *self.desiredOrientation])
        P_f = np.array([0.0, 0.0, 0.05, *self.desiredOrientation])
        now = self.get_clock().now().seconds_nanoseconds()
        # Tiempo actual
        t_i = now[0] + now[1] * 1e-9
        # Duración de la maniobra en segundos
        mvr_time = 10.0  
        # Derivadas a cero en los extremos de trayectoria
        n_deriv = 2
        period = 1.0 / 100  # 100 Hz
        self.get_logger().info('Iniciando rutina de movimiento suave a la posición inicial')
        t_f = t_i + mvr_time
        next_time = time.time() + period
        hold_time = 5.0
        while rclpy.ok():
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
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
        # Imprime pose actual
        # self.get_logger().info(f'Pose actual: {self.last_synced_pose}')

    def get_in_contact(self):
        """
        Mueve el robot suavemente a la posición de contacto usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /synced_data
        while rclpy.ok() and self.last_synced_pose is None:
            self.get_logger().info('Esperando datos de /synced_data para obtener la pose de contacto...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        current_pose = self.last_synced_pose  # [x, y, z, qx, qy, qz, qw]
        # Imprimir current_pose
        self.get_logger().info(f'Pose actual antes de mover a contacto: {current_pose}')
        # Para la interpolación solo usamos las componentes de posición
        P_i = current_pose[0:3]
        # Pose de contacto deseada con las coordenadas x,y iguales a las iniciales P_i
        A = -0.0489
        B = -0.1113
        C = 10.0303 
        D = 0.2506
        desiresZ = -(A*P_i[0] + B*P_i[1] + D)/C
        P_f = np.array([P_i[0], P_i[1], desiresZ])
        # Imprimir P_f
        self.get_logger().info(f'Pose de contacto deseada: {P_f}')
        now = self.get_clock().now().seconds_nanoseconds()
        # Tiempo actual
        t_i = now[0] + now[1] * 1e-9
        # Duración de la maniobra en segundos
        mvr_time = 10.0
        # Derivadas a cero en los extremos de trayectoria
        n_deriv = 2
        period = 1.0 / 100  # 100 Hz
        self.get_logger().info('Iniciando rutina de movimiento suave a la posición de contacto')
        t_f = t_i + mvr_time
        next_time = time.time() + period
        hold_time = 5.0
        while rclpy.ok():
            now = self.get_clock().now().seconds_nanoseconds()
            t_now = now[0] + now[1] * 1e-9
            if t_now > t_f + hold_time:
                break
            # Interpolación solo en posición
            X = self.rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t_now, n_deriv)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(X[0])
            pose_msg.pose.position.y = float(X[1])
            pose_msg.pose.position.z = float(X[2])
            pose_msg.pose.orientation.x = self.desiredQuat[0]
            pose_msg.pose.orientation.y = self.desiredQuat[1]
            pose_msg.pose.orientation.z = self.desiredQuat[2]
            pose_msg.pose.orientation.w = self.desiredQuat[3]
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
        self.get_logger().info('Rutina de movimiento completada')

    def lift_end_effector(self):
        """
        Mueve el robot suavemente a la posición elevada usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /synced_data
        while rclpy.ok() and self.last_synced_pose is None:
            self.get_logger().info('Esperando datos de /synced_data para obtener la pose de contacto...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        P_i = self.last_synced_pose  # [x, y, z, qx, qy, qz, qw]
        # Para la interpolación solo usamos las componentes de posición
        P_i = P_i[0:3]
        # Pose de contacto deseada con las coordenadas x,y iguales a las iniciales P_i
        P_f = np.array([P_i[0], P_i[1], 0.05])
        now = self.get_clock().now().seconds_nanoseconds()
        # Tiempo actual
        t_i = now[0] + now[1] * 1e-9
        # Duración de la maniobra en segundos
        mvr_time = 10.0
        # Derivadas a cero en los extremos de trayectoria
        n_deriv = 2
        period = 1.0 / 100  # 100 Hz
        self.get_logger().info('Iniciando rutina de movimiento suave a la posición elevada')
        t_f = t_i + mvr_time
        next_time = time.time() + period
        hold_time = 5.0
        while rclpy.ok():
            now = self.get_clock().now().seconds_nanoseconds()
            t_now = now[0] + now[1] * 1e-9
            if t_now > t_f + hold_time:
                break
            # Interpolación solo en posición
            X = self.rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t_now, n_deriv)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(X[0])
            pose_msg.pose.position.y = float(X[1])
            pose_msg.pose.position.z = float(X[2])
            pose_msg.pose.orientation.x = self.desiredQuat[0]
            pose_msg.pose.orientation.y = self.desiredQuat[1]
            pose_msg.pose.orientation.z = self.desiredQuat[2]
            pose_msg.pose.orientation.w = self.desiredQuat[3]
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
        self.get_logger().info('Rutina de movimiento completada')

    def execute_motion(self):
        self.get_logger().info('Ejecutando rutina de movimiento')
        # Llama a la rutina de movimiento
        self.move_to_initial_pose()
        self.get_in_contact()
        # Empieza a grabar los datos de /synced_data
        # Lee la trayectoria y empieza el movimiento en el plano
        # Deja de grabar los datos de /synced_data
        self.lift_end_effector()

def main(args=None):
    rclpy.init(args=args)
    node = URMotionRoutinesNode()
    node.execute_motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()