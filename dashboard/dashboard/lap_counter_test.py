import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from std_msgs.msg import Int32  # Import the Int32 message type for lap count
import numpy as np
import csv
from math import radians, cos, sin, sqrt, atan2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import os
from ament_index_python.packages import get_package_share_directory


package_share_directory = get_package_share_directory('dashboard')
waypoints_dir = os.path.join(package_share_directory, 'waypoints')

class LapCounterNode(Node):

    def __init__(self):
        super().__init__('lap_counter_node')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize variables
        self.origin_latitude = 37.561672
        self.origin_longitude = 126.936926
        self.distance_threshold = 10.0
        self.lap_count = 0
        self.target_index = 0
        self.reference_path = self.load_reference_path(os.path.join(waypoints_dir, 'yonsei_eng_manual.csv'))
        self.target_position = self.reference_path[self.target_index]
        
        # Subscribe to the GPS topic
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_callback,
            qos_profile
        )

        # Create a publisher for the lap count
        self.lap_count_publisher = self.create_publisher(
            Int32,
            '/lap_count',
            qos_profile
        )

        self.get_logger().info('LapCounterNode has been started.')

    def load_reference_path(self, filepath):
        path = []
        with open(filepath, mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                x, y = float(row[0]), float(row[1])
                path.append((x, y))
        return path

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters
        phi1 = radians(lat1)
        phi2 = radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)

        a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c

    def gps_to_local_coordinates(self, lat, lon):
        x = self.haversine_distance(self.origin_latitude, self.origin_longitude, self.origin_latitude, lon) * (1 if lon > self.origin_longitude else -1)
        y = self.haversine_distance(self.origin_latitude, self.origin_longitude, lat, self.origin_longitude) * (1 if lat > self.origin_latitude else -1)
        return (x, y)

    def calculate_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def gps_callback(self, msg):
        current_position = self.gps_to_local_coordinates(msg.lat, msg.lon)
        distance = self.calculate_distance(current_position, self.target_position)

        # Check if the current position is within the threshold distance to the target position
        if distance <= self.distance_threshold:
            self.target_index += 1

            # If we've reached the end of the path, increase the lap count and reset the target index
            if self.target_index >= len(self.reference_path):
                self.lap_count += 1
                self.get_logger().info(f'Lap {self.lap_count} completed!')
                self.target_index = 0

                # Publish the new lap count
                lap_count_msg = Int32()
                lap_count_msg.data = self.lap_count
                self.lap_count_publisher.publish(lap_count_msg)

            self.target_position = self.reference_path[self.target_index]


def main(args=None):
    rclpy.init(args=args)
    node = LapCounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
