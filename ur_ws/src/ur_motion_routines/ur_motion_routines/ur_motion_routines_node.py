import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_interfaces.msg import SyncData
import numpy as np
from math import factorial
import time

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
            0.7071,
            -0.7071,
            -0.0022,
            -0.0056
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
            
    def rest_to_rest_trajectory(self, P_i, P_f, t_i, mvr_time, t, n, *, clamp=False, return_progress=False):
        """ Genera una trayectoria rest-to-rest (polinómica) entre dos poses P_i y P_f
        imponiendo que las primeras n derivadas (posición, velocidad, ..., hasta orden n-1)
        sean cero al inicio y al final.

        Parámetros
        ----------
        P_i : array-like (dim,)
            Pose inicial (vector).
        P_f : array-like (dim,)
            Pose final (vector).
        t_i : float
            Tiempo inicial del maniobra.
        mvr_time : float
            Duración total de la maniobra (t_f = t_i + mvr_time).
        t : float
            Tiempo actual en el que se evalúa la trayectoria.
        n : int
            Número de derivadas (empezando por la 0) que se fuerzan a cero en inicio y fin.
            Debe ser >= 1.
        clamp : bool (opcional, False por defecto)
            Si True, se fuerza t a permanecer en [t_i, t_f] (saturación).
        return_progress : bool (opcional, False por defecto)
            Si True, también devuelve el escalar de progreso f(t) en [0,1] (idealmente).

        Retorna
        -------
        X : np.ndarray (dim,)
            Pose interpolada en el tiempo t.
        f : float (opcional)
            Escalar de interpolación (solo si return_progress=True).

        Notas
        -----
        - La implementación replica la lógica original de MATLAB.
        - Se resuelve un sistema lineal A * Alpha = Lambda para obtener los coeficientes.
        - Para múltiples evaluaciones en distintos t con mismos parámetros, conviene
        precomputar Alpha (ver función auxiliar más abajo).
        """
        if n < 1:
            raise ValueError("Hijole mano, no se va a poder xd (n debe ser >= 1)")

        P_i = np.asarray(P_i, dtype=float)
        P_f = np.asarray(P_f, dtype=float)

        if P_i.shape != P_f.shape:
            raise ValueError("P_i y P_f deben tener la misma dimensión")

        t_f = t_i + mvr_time
        if mvr_time <= 0:
            raise ValueError("mvr_time debe ser positivo")

        # Normalización de tiempo
        if clamp:
            if t <= t_i:
                if return_progress:
                    return P_i.copy(), 0.0
                return P_i.copy()
            if t >= t_f:
                if return_progress:
                    return P_f.copy(), 1.0
                return P_f.copy()

        s = (t - t_i) / (t_f - t_i)

        # Construcción de la matriz A (n+1)x(n+1)
        # En MATLAB: indices i,j van de 1 a n+1
        # Aquí usamos i,j de 0 a n (ajustando factoriales)
        A = np.empty((n + 1, n + 1), dtype=float)
        for i in range(n + 1):
            for j in range(n + 1):
                # MATLAB: A(i,j) = (-1)^(j-1) * factorial(n+j) / ((t_f-t_i)^(i-1) * factorial(n+j-i+1))
                # Ajuste a base 0: j_mat = j+1, i_mat = i+1
                # factorial(n + (j+1)) -> factorial(n + j + 1)
                # (t_f - t_i)^(i_mat - 1) -> (t_f - t_i)^i
                # factorial(n + (j+1) - (i+1) + 1) = factorial(n + j - i + 1)
                A[i, j] = ((-1) ** j) * factorial(n + j + 1) / ((t_f - t_i) ** i * factorial(n + j - i + 1))

        # Vector Lambda = [1, 0, 0, ..., 0]^T
        Lambda = np.zeros(n + 1)
        Lambda[0] = 1.0

        # Resolver para Alpha
        Alpha = np.linalg.solve(A, Lambda)

        # f(s) = sum_{j=0}^n (-1)^j * Alpha[j] * s^(n+1+j)
        powers = s ** (np.arange(n + 1) + (n + 1))
        signs = (-1) ** np.arange(n + 1)
        f = np.sum(signs * Alpha * powers)

        # Trayectoria por cada dimensión (f es escalar común)
        X = P_i + (P_f - P_i) * f

        if return_progress:
            return X, f
        return X


    def precompute_alpha(self, n, t_i, mvr_time):
        """
        Precomputa Alpha para un dado n y duración (útil si se evaluará muchas veces).
        Devuelve Alpha y una función 'evaluate(s)' que da f(s).
        """
        if n < 1:
            raise ValueError("n debe ser >= 1")
        t_f = t_i + mvr_time
        if mvr_time <= 0:
            raise ValueError("mvr_time debe ser positivo")

        A = np.empty((n + 1, n + 1), dtype=float)
        for i in range(n + 1):
            for j in range(n + 1):
                A[i, j] = ((-1) ** j) * factorial(n + j + 1) / ((t_f - t_i) ** i * factorial(n + j - i + 1))
        Lambda = np.zeros(n + 1)
        Lambda[0] = 1.0
        Alpha = np.linalg.solve(A, Lambda)

        signs = (-1) ** np.arange(n + 1)
        exponents = (n + 1) + np.arange(n + 1)

        def evaluate_s(s):
            s = np.asarray(s)
            return np.sum(signs * Alpha * (s[..., None] ** exponents), axis=-1)

        return Alpha, evaluate_s
    
    def move_to_initial_pose(self):
        """
        Mueve el robot suavemente a una posición inicial cartesiana usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /synced_data
        while rclpy.ok() and self.last_synced_pose is None:
            self.get_logger().info('Esperando datos de /synced_data para obtener la pose inicial...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        P_i = self.last_synced_pose.copy()  # [x, y, z, qx, qy, qz, qw]
        # Pose inicial deseada
        # P_f = np.array([0.03, 0.11, 0.1, *self.desiredOrientation])
        P_f = np.array([0.10, 0.1, 0.1, *self.desiredOrientation])
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
        while rclpy.ok():
            now = self.get_clock().now().seconds_nanoseconds()
            t_now = now[0] + now[1] * 1e-9
            if t_now > t_f:
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
        self.get_logger().info('Rutina de movimiento completada')
    
    def simple_routine(self):
        self.get_logger().info('Ejecutando rutina de movimiento simple')
        # Llama a la rutina de movimiento
        self.move_to_initial_pose()

def main(args=None):
    rclpy.init(args=args)
    node = URMotionRoutinesNode()
    node.simple_routine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()