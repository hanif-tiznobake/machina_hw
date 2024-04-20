import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import WrenchStamped
from sensor_interfaces.srv import GetSensorData
import queue
import time

class SensorClient(Node):
    def __init__(self):
        super().__init__('sensor_service_client')
        self.declare_parameter('server_name_1', 'sensor_1')
        self.declare_parameter('server_name_2', 'sensor_2')
        self.declare_parameter('frequency', 200)

    def run(self):
        self.request_frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        
        self.client1 = self.create_client(GetSensorData, self.get_parameter('server_name_1').get_parameter_value().string_value)
        self.client2 = self.create_client(GetSensorData, self.get_parameter('server_name_2').get_parameter_value().string_value)
        self.publisher = self.create_publisher(WrenchStamped, 'sensor_data', 10)

        self.data_queue = queue.Queue(maxsize=10)
        self.last_published_data = None
        self.ensure_service_availability()
        self.publish_timer = self.create_timer(1/500, self.publish_data)
        self.request_timer = self.create_timer(1 / self.request_frequency, self.timer_callback)
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()

    def ensure_service_availability(self):
        self.service1_available = False
        self.service2_available = False
        
        while not self.service1_available or not self.service2_available:
            if not self.service1_available:
                self.service1_available = self.client1.wait_for_service(timeout_sec=1.0)
                if not self.service1_available:
                    self.get_logger().warn('Waiting for Sensor Service 1 to become available...')
            
            if not self.service2_available:
                self.service2_available = self.client2.wait_for_service(timeout_sec=1.0)
                if not self.service2_available:
                    self.get_logger().warn('Waiting for Sensor Service 2 to become available...')
            
            time.sleep(1)

    def timer_callback(self):
        future1 = self.client1.call_async(GetSensorData.Request())
        future2 = self.client2.call_async(GetSensorData.Request())

        future1.add_done_callback(self.handle_service_response1)
        future2.add_done_callback(self.handle_service_response2)

    def publish_data(self):
        if not self.data_queue.empty():
            self.last_published_data = self.data_queue.get()
        if self.last_published_data:
            self.publisher.publish(self.last_published_data)

    def handle_service_response1(self, future):
        try:
            result = future.result()
            if result is not None and result.header.frame_id != '':
                d = WrenchStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id=result.header.frame_id), wrench=result.data)
                self.data_queue.put(d)
            else:
                self.get_logger().warn('Empty data received from Sensor 1')
        except Exception as e:
            self.get_logger().error(f'Error receiving data from Sensor 1: {e}')
            self.service1_available = False
            self.ensure_service_availability()

    def handle_service_response2(self, future):
        try:
            result = future.result()
            if result is not None and result.header.frame_id != '':
                d = WrenchStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id=result.header.frame_id), wrench=result.data)
                self.data_queue.put(d)
            else:
                self.get_logger().warn('Empty data received from Sensor 2')
        except Exception as e:
            self.get_logger().error(f'Error receiving data from Sensor 2: {e}')
            self.service2_available = False
            self.ensure_service_availability()

def main(args=None):
    rclpy.init(args=args)
    sensor_data_client = SensorClient()
    sensor_data_client.run()

if __name__ == '__main__':
    main()
