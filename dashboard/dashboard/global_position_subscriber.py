import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition

class GlobalPositionSubscriber(Node):
    def __init__(self):
        super().__init__('global_position_subscriber')
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        latitude = msg.lat[0]
        longitude = msg.lon[0]
        self.get_logger().info(f'Latitude: {latitude:.6f}, Longitude: {longitude:.6f}')

def main(args=None):
    rclpy.init(args=args)
    global_position_subscriber = GlobalPositionSubscriber()
    rclpy.spin(global_position_subscriber)
    global_position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
