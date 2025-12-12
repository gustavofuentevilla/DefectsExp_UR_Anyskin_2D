import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from custom_interfaces.msg import SyncData
from std_srvs.srv import Trigger
import numpy as np
from math import factorial
import time
import subprocess
import signal
import os
from datetime import datetime
import shutil
import threading

# Commentarios:
# Posibles parámetros a ajustar desde el launch file:
# - Duración de las maniobras (mvr_time)
# - Frecuencia de publicación (period)
# - hold_time en cada posición final

class URMotionRoutinesNode(Node):
    def __init__(self):
        super().__init__('ur_motion_routines_node')
        self.get_logger().info('Nodo de rutinas de movimiento iniciado')
        # declare and read launch parameter 'i' (integer). Default 1 if not provided.
        try:
            self.declare_parameter('i', 1)
            iteration_val = self.get_parameter('i').value
            try:
                # attempt to coerce to int if it's a string
                self.iteration = int(iteration_val)
            except Exception:
                self.iteration = iteration_val
            self.get_logger().info(f"Parameter 'iteration' = {self.iteration}")
        except Exception:
            # parameter system may fail in odd contexts; fall back
            self.iteration = 1
        # Publisher para el controlador cartesiano
        self.target_pose_pub = self.create_publisher(PoseStamped, '/cartesian_compliance_controller/target_frame', 10)
        # Publisher for target wrenches (forces/torques)
        self.target_wrench_pub = self.create_publisher(WrenchStamped, '/cartesian_compliance_controller/target_wrench', 10)
        # Subscriber a /synced_data
        self.last_synced_pose = None
        self.last_pose = None
        self.synced_data_sub = self.create_subscription(
            SyncData,
            '/synced_data',
            self.synced_data_callback,
            10
        )
        # Also subscribe directly to the fast pose topic to always have the latest pose
        # This helps if /synced_data is published only sporadically (sensor-triggered)
        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            '/ee_pose_fast',
            self.ee_pose_callback,
            10
        )
        # Client for resetting sensor baseline (std_srvs/Trigger). Create once and reuse.
        try:
            self.reset_baseline_client = self.create_client(Trigger, '/reset_baseline')
        except Exception:
            # keep attribute even if creation failed for any reason
            self.reset_baseline_client = None
        self.desiredOrientation = np.array([
            0.7071,
            -0.7071,
            0.0000,
            0.0000
        ])
        #0.0002, -0.0041
        self.desiredQuat = self.desiredOrientation.copy()
        self.last_Orientation = None
        # default target wrench (Ceros): [fx, fy, fz, tx, ty, tz]
        self.default_target_wrench = np.zeros(6, dtype=float)
        # ros2 bag subprocess handle (initialized when needed)
        self._rosbag_proc = None
        self._rosbag_dir = None
        # file handle for ros2 bag stdout/stderr
        self._rosbag_log_file = None
        self._rosbag_log_path = None
        # in-memory recording buffer (alternative to rosbag)
        self._recording = False
        self._record_buffer = []
        # output directory for CSV recordings and trajectory readings
        self._record_output_dir = '/home/gustavo-fuentevilla/DefectsExp_UR/Tests/N100/2Def/Test'
        self._trajectory_dir = '/home/gustavo-fuentevilla/DefectsExp_UR/Tests/N100/2Def/Test'
        self.desiredZ = None
        

    def start_rosbag(self, topics, output_dir=None):
        """
        Start recording the given topics with `ros2 bag record` in a subprocess.
        topics: list of topic names (strings)
        output_dir: optional path where the bag folder will be created (defaults to cwd)
        """
        if self._rosbag_proc is not None:
            raise RuntimeError('rosbag already running')
        if output_dir is None:
            output_dir = os.getcwd()
        # ensure output_dir exists and is writable
        try:
            os.makedirs(output_dir, exist_ok=True)
        except Exception:
            self.get_logger().warn(f'Could not create output_dir {output_dir}, using cwd')
            output_dir = os.getcwd()
        if not os.access(output_dir, os.W_OK):
            self.get_logger().warn(f'Output directory {output_dir} is not writable, using cwd')
            output_dir = os.getcwd()

        # quick check that 'ros2' is available
        if shutil.which('ros2') is None:
            raise RuntimeError("'ros2' executable not found in PATH for subprocess. Ensure ROS 2 is sourced in this environment.")
        # include microseconds to reduce chance of name collision
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        base_name = f'rosbag_{ts}'
        # choose a unique name that does not exist yet, but DO NOT create it; ros2 will create it
        bag_dir = os.path.join(output_dir, base_name)
        if os.path.exists(bag_dir):
            suffix = 1
            while True:
                candidate = f"{bag_dir}_{suffix}"
                if not os.path.exists(candidate):
                    bag_dir = candidate
                    break
                suffix += 1
        # use --topics flag to avoid positional-argument deprecation warning
        cmd = ['ros2', 'bag', 'record', '-o', bag_dir, '--topics'] + list(topics)
        # start subprocess in its own process group so we can kill the whole group later
        # capture stdout/stderr to a log file in the output_dir (not inside bag_dir which ros2 creates)
        log_path = os.path.join(output_dir, f'{base_name}.ros2_bag_record.log')
        # try to open the log file; if that fails, fall back to DEVNULL but continue
        try:
            log_file = open(log_path, 'a+', encoding='utf-8')
            stdout_dest = log_file
            stderr_dest = log_file
            self._rosbag_log_file = log_file
            self._rosbag_log_path = log_path
        except Exception as e:
            self.get_logger().warn(f'Could not open ros2 bag log file {log_path}: {e}. Falling back to DEVNULL.')
            stdout_dest = subprocess.DEVNULL
            stderr_dest = subprocess.DEVNULL
            self._rosbag_log_file = None
            self._rosbag_log_path = None

        try:
            self._rosbag_proc = subprocess.Popen(cmd, preexec_fn=os.setpgrp, stdout=stdout_dest, stderr=stderr_dest)
        except Exception:
            # close file if process couldn't be started
            try:
                if self._rosbag_log_file is not None:
                    self._rosbag_log_file.close()
            except Exception:
                pass
            self._rosbag_log_file = None
            self._rosbag_log_path = None
            raise
        self._rosbag_dir = bag_dir
        self.get_logger().info(f'Started ros2 bag recording to {bag_dir} topics={topics} (log: {log_path})')
        return bag_dir

    def start_rosbag_and_wait(self, topics, output_dir=None, timeout=10.0):
        """
        Start ros2 bag and wait until files appear in the bag folder or timeout.
        Returns the bag_dir when ready. Raises RuntimeError/TimeoutError on failure.
        """
        # Try launching ros2 bag multiple times if we detect 'output folder already exists' errors
        max_attempts = 5
        attempt = 0
        last_exc = None
        while attempt < max_attempts:
            attempt += 1
            bag_dir = None
            try:
                bag_dir = self.start_rosbag(topics, output_dir=output_dir)
            except Exception as e:
                last_exc = e
                # if start_rosbag raised because 'ros2' is not in PATH or other fatal error, stop retrying
                if "'ros2' executable not found" in str(e):
                    raise
                # otherwise try again with a new timestamp (loop will retry)
                self.get_logger().warn(f'Attempt {attempt} to start ros2 bag failed at launch: {e}')
                # small backoff
                time.sleep(0.1 * attempt)
                continue

            # wait until the bag directory contains files (ros2 bag creates files quickly)
            t0 = time.time()
            while True:
                # if process died, abort and inspect log to decide whether to retry
                if self._rosbag_proc.poll() is not None:
                    # flush buffered output
                    try:
                        if self._rosbag_log_file is not None:
                            self._rosbag_log_file.flush()
                    except Exception:
                        pass
                    log_snippet = ''
                    try:
                        if self._rosbag_log_path and os.path.exists(self._rosbag_log_path):
                            with open(self._rosbag_log_path, 'r', encoding='utf-8', errors='replace') as lf:
                                lf.seek(0, os.SEEK_END)
                                size = lf.tell()
                                lf.seek(max(0, size - 8192))
                                log_snippet = lf.read()
                    except Exception:
                        log_snippet = '<failed to read ros2 bag log>'
                    code = self._rosbag_proc.returncode
                    # if log indicates output folder already exists, try again with a new name
                    if 'already exists' in log_snippet or 'Output folder' in log_snippet:
                        try:
                            self.stop_rosbag()
                        except Exception:
                            pass
                        self.get_logger().warn(f'ros2 bag reported existing output folder; retrying (attempt {attempt}/{max_attempts})')
                        time.sleep(0.1 * attempt)
                        break  # break inner loop to retry
                    else:
                        # non-recoverable: include log and raise
                        raise RuntimeError(f'ros2 bag process exited unexpectedly (returncode={code}). log:\n{log_snippet}')

                try:
                    entries = os.listdir(bag_dir)
                except FileNotFoundError:
                    entries = []

                if entries:
                    # Check that at least one file in the bag folder is actually being written to.
                    # Some ros2 bag implementations create zero-byte metadata files before writing data.
                    # We consider recording started when we detect file size growth on any file,
                    # or when a file already has a non-trivial size (likely data) to avoid missing
                    # short-lived recording windows where growth may not be observed in a tiny interval.
                    files = [os.path.join(bag_dir, e) for e in entries if not e.startswith('.')]
                    recording_detected = False
                    min_bytes_considered_recording = 512
                    for fp in files:
                        try:
                            s1 = os.path.getsize(fp)
                        except Exception:
                            s1 = 0

                        # If the file already has a reasonable size, accept it as recording.
                        if s1 >= min_bytes_considered_recording:
                            self.get_logger().info(f'ros2 bag file {fp} size={s1} >= {min_bytes_considered_recording} -> assuming recording')
                            recording_detected = True
                            break

                        # Otherwise, attempt to detect active writing by observing growth over a short interval
                        try:
                            time.sleep(0.15)
                            s2 = os.path.getsize(fp)
                        except Exception:
                            s2 = s1

                        if s2 > s1:
                            self.get_logger().info(f'ros2 bag file {fp} grew {s1} -> {s2} bytes; recording detected')
                            recording_detected = True
                            break

                        # If file is zero and didn't grow, try one more short probe
                        if s1 == 0 and s2 == 0:
                            try:
                                time.sleep(0.15)
                                s3 = os.path.getsize(fp)
                            except Exception:
                                s3 = 0
                            if s3 > s2:
                                self.get_logger().info(f'ros2 bag file {fp} grew on second probe {s2} -> {s3} bytes; recording detected')
                                recording_detected = True
                                break

                    if recording_detected:
                        self.get_logger().info('ros2 bag appears to be recording (confirmed by file size or growth)')
                        return bag_dir

                if time.time() - t0 > timeout:
                    # timeout: stop process and raise
                    try:
                        self.stop_rosbag()
                    except Exception:
                        pass
                    raise TimeoutError(f'ros2 bag did not start recording within {timeout} seconds')
                time.sleep(0.1)

        # exhausted retries
        raise RuntimeError(f'Failed to start ros2 bag after {max_attempts} attempts: last error: {last_exc}')

    def stop_rosbag(self):
        """
        Stop the ros2 bag subprocess if running and wait for it to finish.
        """
        if self._rosbag_proc is None:
            return
        try:
            # send SIGINT to process group to gracefully stop recording
            os.killpg(os.getpgid(self._rosbag_proc.pid), signal.SIGINT)
        except Exception:
            try:
                self._rosbag_proc.terminate()
            except Exception:
                pass
        try:
            self._rosbag_proc.wait(timeout=5)
        except Exception:
            # give up waiting
            pass
        # close log file if we opened one
        try:
            if self._rosbag_log_file is not None:
                try:
                    self._rosbag_log_file.flush()
                except Exception:
                    pass
                try:
                    self._rosbag_log_file.close()
                except Exception:
                    pass
        finally:
            self._rosbag_log_file = None
            self._rosbag_log_path = None
        self.get_logger().info(f'ros2 bag recording stopped, data in {self._rosbag_dir}')
        self._rosbag_proc = None
        self._rosbag_dir = None

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
        # Verifica que la pose se está actualizando
        # print("self.last_synced_pose:", self.last_synced_pose)
        # If in-memory recording is enabled, append a snapshot
        try:
            if getattr(self, '_recording', False):
                # stamp: builtin_interfaces/Time
                stamp = msg.stamp
                t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                meas = []
                try:
                    meas = list(msg.measurements.data)
                except Exception:
                    meas = []
                row = {
                    't': t,
                    'px': float(pose.position.x),
                    'py': float(pose.position.y),
                    'pz': float(pose.position.z),
                    'qx': float(pose.orientation.x),
                    'qy': float(pose.orientation.y),
                    'qz': float(pose.orientation.z),
                    'qw': float(pose.orientation.w),
                    'measurements': meas,
                }
                self._record_buffer.append(row)
        except Exception:
            # don't let recording errors break the callback
            pass

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

        A = -0.00857
        B = -0.007878
        C = 1.428
        D = 0.02975
        self.desiredZ = -(A*pose.position.x + B*pose.position.y + D)/C + 0.0025
        #-0.0175
            
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

    def rest_to_rest_trajectory2(self, P_i, P_f, t_i, mvr_time, t, n):
        """
        Correct rest-to-rest interpolation.

        Construct the minimal-degree polynomial p(tau) with tau in [0,1], degree m=2n+1,
        that satisfies p(0)=0 and derivatives up to order n vanish at tau=0,
        and p(1)=1 with derivatives up to order n vanish at tau=1.

        The trajectory is X = P_i + (P_f - P_i) * p(tau).
        """
        if n < 1:
            raise ValueError("El orden de derivadas debe ser mayor o igual a 1")

        P_i = np.array(P_i, dtype=float)
        P_f = np.array(P_f, dtype=float)

        t_f = t_i + mvr_time

        # Boundary handling
        if t < t_i:
            return P_i
        if t > t_f:
            return P_f

        tau = (t - t_i) / (t_f - t_i)

        # degree and unknown coefficients
        m = 2 * n + 1
        unknown_count = n + 1  # coefficients c_{n+1}..c_m

        # Build linear system at tau=1 for derivatives r=0..n
        M = np.zeros((unknown_count, unknown_count), dtype=float)
        b = np.zeros((unknown_count,), dtype=float)
        b[0] = 1.0

        for r in range(unknown_count):
            for j in range(unknown_count):
                k = n + 1 + j
                if k < r:
                    M[r, j] = 0.0
                else:
                    # falling factorial k*(k-1)*...*(k-r+1) = k!/(k-r)!
                    M[r, j] = float(factorial(k) // factorial(k - r))

        try:
            coeffs_unknown = np.linalg.solve(M, b)
        except np.linalg.LinAlgError:
            coeffs_unknown, *_ = np.linalg.lstsq(M, b, rcond=None)

        coeffs = np.zeros((m + 1,), dtype=float)
        for j in range(unknown_count):
            k = n + 1 + j
            coeffs[k] = coeffs_unknown[j]

        powers = np.array([tau ** k for k in range(m + 1)], dtype=float)
        p = float(np.dot(coeffs, powers))

        X = P_i + (P_f - P_i) * p
        return X

    def publish_target_wrench(self, wrench=None, frame_id="ee_link"):
        """
        Publish a target wrench (WrenchStamped) to the controller topic.

        wrench: array-like with 6 elements [fx,fy,fz,tx,ty,tz]. If None, uses self.target_wrench.
        """
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        w = np.array(wrench, dtype=float) if wrench is not None else self.default_target_wrench
        # force
        msg.wrench.force.x = float(w[0])
        msg.wrench.force.y = float(w[1])
        msg.wrench.force.z = float(w[2])
        # torque
        msg.wrench.torque.x = float(w[3])
        msg.wrench.torque.y = float(w[4])
        msg.wrench.torque.z = float(w[5])
        self.target_wrench_pub.publish(msg)

    def move_to_initial_pose(self):
        """
        Mueve el robot suavemente a una posición inicial cartesiana usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /synced_data
        while rclpy.ok() and self.last_pose is None:
            self.get_logger().info('Esperando datos de /synced_data para obtener la pose inicial...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        P_i = self.last_pose  # [x, y, z, qx, qy, qz, qw]
        # Pose inicial deseada (leida desde la primera trayectoria)
        trajectory_file = f'{self._trajectory_dir}/trayectoria_{self.iteration}.csv'
        x0, y0 = np.loadtxt(trajectory_file, delimiter=",", skiprows=1, max_rows=1, usecols=(1,2), unpack=True)
        P_f = np.array([x0, y0, 0.05, *self.desiredQuat])
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
        hold_time = 10.0
        while rclpy.ok():
            # process incoming messages so subscriber callbacks update last_pose
            # rclpy.spin_once(self, timeout_sec=0.0)
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
            self.publish_target_wrench()
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
        # Imprime pose actual
        # self.get_logger().info(f'Pose final initial pose: {self.last_pose}')

    def get_in_contact(self):
        """
        Mueve el robot suavemente a la posición de contacto usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /ee_pose_fast
        while rclpy.ok() and self.last_pose is None:
            self.get_logger().info('Esperando datos de /ee_pose_fast para obtener la pose de contacto...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        current_pose = self.last_pose  # [x, y, z, qx, qy, qz, qw]
        # Imprimir current_pose
        # self.get_logger().info(f'Pose actual antes de mover a contacto: {current_pose}')
        # Para la interpolación solo usamos las componentes de posición
        P_i = current_pose[0:3]
        # self.get_logger().info(f'Pi: {P_i}')
        # Pose de contacto deseada con las coordenadas x,y iguales a las iniciales P_i
        # A = -0.00857
        # B = -0.007878
        # C = 1.428
        # D = 0.02975
        # desiredZ = -(A*P_i[0] + B*P_i[1] + D)/C
        P_f = np.array([current_pose[0], current_pose[1], self.desiredZ]) #-0.0193
        # Imprimir P_f
        self.get_logger().info(f'Pf: {P_f}')
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
        hold_time = 10.0
        while rclpy.ok():
            # process incoming messages so subscriber callbacks update last_pose
            rclpy.spin_once(self, timeout_sec=0.0)
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
            # publish target wrench (can be zero or customized)
            self.publish_target_wrench([0,0,5,0,0,0], frame_id="ee_link")
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
            # self.get_logger().info(f'X: {X}')
        self.get_logger().info('Rutina de movimiento completada')

    def lift_end_effector(self):
        """
        Mueve el robot suavemente a la posición elevada usando interpolación rest-to-rest.
        """
        # Esperar a tener una lectura válida de /synced_data
        while rclpy.ok() and self.last_pose is None:
            self.get_logger().info('Esperando datos de /synced_data para obtener la pose de contacto...')
            rclpy.spin_once(self, timeout_sec=0.1)
        # Pose actual del robot
        current_pose = self.last_pose  # [x, y, z, qx, qy, qz, qw]
        # Para la interpolación solo usamos las componentes de posición
        P_i = current_pose[0:3]
        # Pose de contacto deseada con las coordenadas x,y iguales a las iniciales P_i
        P_f = np.array([current_pose[0], current_pose[1], 0.05])
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
            # process incoming messages so subscriber callbacks update last_synced_pose
            rclpy.spin_once(self, timeout_sec=0.0)
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
            # publish target wrench (can be zero or customized)
            self.publish_target_wrench()
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
        self.get_logger().info('Rutina de movimiento completada')

    def ergodic_motion(self):
        """
        Ejecuta el movimiento ergódico leyendo la trayectoria desde un archivo.
        """
        trajectory_file = f'{self._trajectory_dir}/trayectoria_{self.iteration}.csv'
        # Carga la trayectoria desde el archivo csv
        trajectory = np.loadtxt(trajectory_file, delimiter=',', skiprows=1, usecols=(1,2), unpack=True)
        num_points = trajectory.shape[1]
        period = 1.0 / 100  # 100 Hz
        self.get_logger().info('Iniciando rutina de movimiento ergódico')
        next_time = time.time() + period
        for i in range(num_points):
            if not rclpy.ok():
                break
            # process incoming messages so subscriber callbacks update last_synced_pose
            # rclpy.spin_once(self, timeout_sec=0.0)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(trajectory[0, i])
            pose_msg.pose.position.y = float(trajectory[1, i])
            pose_msg.pose.position.z = self.desiredZ #-0.0193
            pose_msg.pose.orientation.x = self.desiredQuat[0]
            pose_msg.pose.orientation.y = self.desiredQuat[1]
            pose_msg.pose.orientation.z = self.desiredQuat[2]
            pose_msg.pose.orientation.w = self.desiredQuat[3]
            # publish target wrench (can be zero or customized)
            self.publish_target_wrench([0,0,5,0,0,0], frame_id="ee_link")
            self.target_pose_pub.publish(pose_msg)
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period
        self.get_logger().info('Rutina de movimiento ergódico completada')

    def start_recording_buffer(self):
        """
        Start in-memory recording of /synced_data into a buffer.
        """

        self._record_buffer = []
        self._recording = True
        self.get_logger().info('Started in-memory recording of /synced_data into buffer')

    def stop_and_save_recording(self, filename_prefix=None):
        """
        Stop in-memory recording and save buffer to CSV. Returns path to CSV file.
        """
        self._recording = False
        buf = list(getattr(self, '_record_buffer', []))
        if not buf:
            self.get_logger().warn('Recording buffer empty; nothing to save')
            return None
        if self._record_output_dir is None:
            outdir = os.getcwd()
        else:
            outdir = self._record_output_dir
        try:
            os.makedirs(outdir, exist_ok=True)
        except Exception:
            outdir = os.getcwd()

        # ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = filename_prefix or 'synced_data'
        out_path = os.path.join(outdir, f'{prefix}_{self.iteration}.csv')

        # Determine maximum measurement length to expand columns
        max_meas = 0
        for r in buf:
            max_meas = max(max_meas, len(r.get('measurements', [])))

        import csv
        with open(out_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # header
            header = ['time_s', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']
            header += [f'm{i}' for i in range(max_meas)]
            writer.writerow(header)
            for r in buf:
                row = [r['t'], r['px'], r['py'], r['pz'], r['qx'], r['qy'], r['qz'], r['qw']]
                meas = r.get('measurements', [])
                # pad measurements
                meas_row = [float(m) for m in meas] + [''] * (max_meas - len(meas))
                row += meas_row
                writer.writerow(row)

        self.get_logger().info(f'Wrote recording buffer to {out_path} ({len(buf)} rows)')
        # clear buffer
        self._record_buffer = []
        return out_path

    def reset_baseline(self, timeout_service: float = 5.0, timeout_response: float = 5.0):
        """
        Call the /reset_baseline service (std_srvs/Trigger) to reset sensor baseline.

        Returns: (success: bool, message: str)
        """
        client = getattr(self, 'reset_baseline_client', None)
        if client is None:
            # try to create it now
            try:
                client = self.create_client(Trigger, '/reset_baseline')
                self.reset_baseline_client = client
            except Exception as e:
                self.get_logger().warn(f'Could not create /reset_baseline client: {e}')
                return False, 'no_client'

        # wait for service to be available
        try:
            if not client.wait_for_service(timeout_sec=timeout_service):
                self.get_logger().warn('/reset_baseline service not available')
                return False, 'service_unavailable'
        except Exception as e:
            self.get_logger().warn(f'Error waiting for /reset_baseline service: {e}')
            return False, 'wait_error'

        # call service and wait for response
        try:
            req = Trigger.Request()
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_response)
            if fut.done():
                resp = fut.result()
                msg = getattr(resp, 'message', '')
                return bool(getattr(resp, 'success', False)), msg
            else:
                self.get_logger().warn('/reset_baseline call timed out')
                return False, 'timeout'
        except Exception as e:
            self.get_logger().warn(f'Exception calling /reset_baseline: {e}')
            return False, 'call_error'

    def execute_motion(self):
        self.get_logger().info('Ejecutando rutina de movimiento')

        # --- Mueve al punto inicial (sólo en la primera ejecución)
        self.move_to_initial_pose()

        # --- Reset anyskin baseline via helper
        try:
            ok, msg = self.reset_baseline(timeout_service=5.0, timeout_response=5.0)
            if ok:
                self.get_logger().info(f"/reset_baseline succeeded: {msg}")
            else:
                self.get_logger().warn(f"/reset_baseline not completed: {msg}")
        except Exception as e:
            self.get_logger().warn(f'Error during reset_baseline helper: {e}')

        # --- Entra en contacto con el plano
        self.get_in_contact()

        # --- Start in-memory recording buffer for /synced_data (will be saved to CSV after motion)
        try:
            self.start_recording_buffer()
        except Exception as e:
            self.get_logger().warn(f'Could not start in-memory recording buffer: {e}. Continuing without recording.')
        
        # --- Lee la trayectoria del archivo y ejecuta movimiento sobre el plano
        self.ergodic_motion()

        # --- Stop in-memory recording and save to CSV
        try:
            csv_path = self.stop_and_save_recording(filename_prefix='synced_data')
            if csv_path:
                self.get_logger().info(f'Saved in-memory recording to {csv_path}')
        except Exception as e:
            self.get_logger().warn(f'Failed to write in-memory recording to CSV: {e}')

        # --- Eleva el efector final
        self.lift_end_effector()

def main(args=None):
    rclpy.init(args=args)
    node = URMotionRoutinesNode()

    # Start spinning in background so subscribers and services are processed concurrently.
    # This lets callbacks run in parallel with execute_motion's blocking loops.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.execute_motion()

    # Shutdown ROS and wait briefly for the spin thread to exit
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    
    node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()