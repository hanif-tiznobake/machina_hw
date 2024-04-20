import rclpy
from rclpy.node import Node
import numpy as np
import socket
from threading import Thread, Lock
import time
from collections import deque
from std_msgs.msg import Header
from geometry_msgs.msg import Wrench, Vector3
from sensor_interfaces.srv import GetSensorData

class SensorFilter:
    def __init__(self, window_size):
        self.values = deque(maxlen=window_size)

    def update(self, new_data):
        self.values.append(new_data)
        return np.mean(self.values, axis=0)

class SensorServer(Node):
    def __init__(self):
        super().__init__('sensor_service_server')
        self.declare_parameter('sensor_name', 'sensor_0')
        self.declare_parameter('sensor_ip', '127.0.0.1')
        self.declare_parameter('sensor_port', 10000)
        self.declare_parameter('frequency', 200)
    
    def run(self):
        self.sensor_name = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.sensor_address = (self.get_parameter('sensor_ip').get_parameter_value().string_value, self.get_parameter('sensor_port').get_parameter_value().integer_value)
        self.freq = int(self.get_parameter('frequency').get_parameter_value().integer_value)
        self.request_size = int(np.ceil(2000/self.freq))
        self.service = self.create_service(GetSensorData, self.sensor_name, self.handle_request)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(self.sensor_address)
        self.filter = SensorFilter(window_size=int(2*self.request_size))
        
        self.lock = Lock()
        self.buffer = deque(maxlen=self.freq)
        self.read_thread = Thread(target=self.read, daemon=True)
        self.read_thread.start()
    
        rclpy.spin(self)
        self.client_socket.close()
        self.destroy_node()
        rclpy.shutdown()

    def read(self):
        while rclpy.ok():
            try:
                self.client_socket.sendall(str(self.request_size).encode())
                byte_data = self.client_socket.recv(8*6*self.request_size*2)
                data = np.frombuffer(byte_data, dtype=np.float64).reshape(-1, 6)
                filtered = np.array([self.filter.update(d) for d in data])
                timestamp = self.get_clock().now().to_msg()
                with self.lock:
                    self.buffer.append((timestamp, filtered))
            except Exception as e:
                self.get_logger().error(f'Failed to read data from sensor: {e}')
            self.get_logger().info(str(len(self.buffer)))
            time.sleep(1/self.freq)

    def handle_request(self, request, response):
        with self.lock:
            if self.buffer:
                timestamp, data = self.buffer.popleft()
                data = np.mean(data, axis=0)

                response.header = Header(stamp=timestamp, frame_id=self.sensor_name)
                response.data = Wrench(
                    force=Vector3(x=data[0], y=data[1], z=data[2]),
                    torque=Vector3(x=data[3], y=data[4], z=data[5])
                )
            else:
                self.get_logger().error('No data available')
        return response        

def main(args=None):
    rclpy.init(args=args)
    node = SensorServer()
    node.run()

if __name__ == '__main__':
    main()
