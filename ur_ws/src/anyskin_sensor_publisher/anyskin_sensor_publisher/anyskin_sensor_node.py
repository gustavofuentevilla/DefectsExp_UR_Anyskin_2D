#!/usr/bin/env python3

# import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Or define a custom message if needed
from std_srvs.srv import Trigger
from anyskin import AnySkinProcess
import numpy as np
import time

class AnyskinSensorPublisher(Node):
    def __init__(self):
        super().__init__('anyskin_sensor_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/anyskin_measurements', 10)
        self.sensor_stream = AnySkinProcess(num_mags=5, port='/dev/ttyACM0')  # Set your port
        self.sensor_stream.start()
        self.baseline = self.get_baseline()
        self.timer = self.create_timer(0.005, self.publish_measurements)  # 200 Hz
        self.get_logger().info("Anyskin Sensor Publisher Node started.")
        self.baseline_srv = self.create_service(Trigger, 'reset_baseline', self.reset_baseline_callback)

    def reset_baseline_callback(self, request, response):
        self.baseline = self.get_baseline()
        response.success = True
        response.message = "Baseline reset successfully."
        return response

    def get_baseline(self):
        # Wait until we get valid baseline data
        for attempt in range(100):  # Try up to 100 times
            baseline_data = self.sensor_stream.get_data(num_samples=5)
            if baseline_data and len(baseline_data) >= 5:
                baseline_data = np.array(baseline_data)[:, 1:]
                baseline = np.mean(baseline_data, axis=0)
                self.get_logger().info(f"Baseline computed: {baseline}")
                return baseline
            else:
                self.get_logger().warn("Waiting for sensor data to compute baseline...")
                time.sleep(0.1)  # Wait 100ms before retrying
        self.get_logger().error("Failed to get baseline data after multiple attempts.")
        return np.zeros(15)  # Or whatever default shape you expect
    
    def publish_measurements(self):
        Datos = self.sensor_stream.get_data(num_samples=1)
        if Datos:
            # sensor_time = Datos[0][0]
            sensor_data = Datos[0][1:]
            # Process the sensor data
            tmp = np.array(sensor_data - self.baseline)
            tmp = tmp.reshape(-1, 3)
            tmp = np.linalg.norm(tmp, axis=1)
            mag_j = np.squeeze(tmp.reshape(1, -1))
            avg_mag = np.mean(mag_j)
            msg = Float32MultiArray()
            msg.data = [float(avg_mag)]  # or publish processed data
            self.publisher_.publish(msg)
            # self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = AnyskinSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.sensor_stream.pause_streaming()
    node.sensor_stream.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()