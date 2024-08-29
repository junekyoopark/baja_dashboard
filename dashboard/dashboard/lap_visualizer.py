import sys
import os
import csv
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget, QPushButton, QMessageBox
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen
from PyQt5.QtCore import Qt, QTimer
from PIL import Image
import requests
from io import BytesIO
import math

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from ament_index_python.packages import get_package_share_directory

# Define global constants for map settings
width = 2000
height = 2000
display_width = 1000
display_height = 1000

latitude = 35.9466289
longitude = 126.5909605
zoom = 18
maptype = 'satellite'
package_share_directory = get_package_share_directory('dashboard')
satellite_map_dir = os.path.join(package_share_directory, 'satellite_map')

def load_naver_api_credentials(file_path):
    credentials = {}
    with open(file_path, 'r') as file:
        for line in file:
            key, value = line.strip().split(' = ')
            credentials[key] = value.strip("'")
    return credentials

def fetch_map_image(latitude, longitude, zoom, maptype, size, CLIENT_ID, CLIENT_SECRET):
    url = f"https://naveropenapi.apigw.ntruss.com/map-static/v2/raster?center={longitude},{latitude}&level={zoom}&w={size.split('x')[0]}&h={size.split('x')[1]}&maptype={maptype}"
    headers = {
        'X-NCP-APIGW-API-KEY-ID': CLIENT_ID,
        'X-NCP-APIGW-API-KEY': CLIENT_SECRET
    }
    try:
        response = requests.get(url, headers=headers)
        response.raise_for_status()  # Raise an error for bad responses (4xx or 5xx)

        image = Image.open(BytesIO(response.content))
        print(os.path.join(satellite_map_dir, 'kunsan.png'))
        image.save(os.path.join(satellite_map_dir, 'kunsan.png'))
        print("Image fetched and saved successfully.")
        return image

    except (requests.exceptions.RequestException, IOError) as e:
        print(f"Error fetching the satellite image: {e}")
        fallback_image_path = os.path.join(satellite_map_dir, 'kunsan.png')
        
        if os.path.exists(fallback_image_path):
            print("Using existing map image as a fallback.")
            return Image.open(fallback_image_path)
        else:
            print("No internet connection and no fallback image available. Exiting...")
            sys.exit(1)

class MapClickApp(QMainWindow):
    def __init__(self, image, latitude, longitude, zoom, csv_file_path):
        super().__init__()

        self.show_warning_message()

        self.latitude = latitude
        self.longitude = longitude
        self.zoom = zoom
        self.csv_file_path = csv_file_path
        self.meters_per_pixel = (40075016.686 * math.cos(math.radians(latitude))) / (2 ** (zoom + 8))
        
        self.setWindowTitle("GUI Path Generator v0.9.1 by JKP")
        self.setGeometry(100, 100, display_width, display_height)

        self.base_image = image.resize((display_width, display_height), Image.ANTIALIAS)
        self.base_image = self.base_image.convert("RGB")
        data = self.base_image.tobytes("raw", "RGB")
        qimage = QImage(data, display_width, display_height, QImage.Format_RGB888)
        self.base_pixmap = QPixmap.fromImage(qimage)

        self.label = QLabel(self)
        self.label.setPixmap(self.base_pixmap.copy())

        self.undo_button = QPushButton("UNDO (UNCLICK)", self)
        self.undo_button.clicked.connect(self.undo_last_action)

        central_widget = QWidget(self)  # Corrected: Create a new central widget
        layout = QVBoxLayout(central_widget)  # Corrected: Set the layout to the new central widget
        layout.addWidget(self.undo_button)
        layout.addWidget(self.label)

        self.setCentralWidget(central_widget)  # Corrected: Set the central widget with the new layout
        
        self.click_history = []

        # Additional history stack to track actions
        self.action_history = []

        self.coordinate_mode = 'latlon'  # or 'meters'

        # Label to show current lat/lon position
        self.position_label = QLabel("Current Position: ", self)
        layout.addWidget(self.position_label)

        # Initialize current position at (0, 0)
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.update_current_position()

        # Initialize ROS 2 node in a separate thread
        self.ros_node = ROSSubscriberNode(self)
        self.ros_node.start_ros_node()

        # Set up a timer to handle ROS 2 spin
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(100)  # Spin ROS every 100 ms

    def show_warning_message(self):
        warning_msg = QMessageBox()
        warning_msg.setIcon(QMessageBox.Information)
        warning_msg.setWindowTitle("A Friendly Reminder :)")
        warning_msg.setText("Ensure your NAVER API credentials are correct and an internet connection is available when launching the program.")
        warning_msg.setInformativeText("You must have proper API credentials to fetch the map.")
        warning_msg.setStandardButtons(QMessageBox.Ok)
        warning_msg.exec_()

    def lat_lon_to_pixel(self, lat, lon):
        """Convert latitude and longitude to pixel coordinates on the map."""
        delta_lat = (lat - self.latitude) * 111320  # Latitude delta in meters
        delta_lon = (lon - self.longitude) * (111320 * math.cos(math.radians(self.latitude)))  # Longitude delta in meters
        
        x = (delta_lon / self.meters_per_pixel) + (display_width / 2)
        y = -(delta_lat / self.meters_per_pixel) + (display_height / 2)
        
        return int(x), int(y)

    def update_current_position(self):
        """Update the map to show the current position as a red dot."""
        x, y = self.lat_lon_to_pixel(self.current_latitude, self.current_longitude)

        # Redraw the base image
        self.label.setPixmap(self.base_pixmap.copy())
        painter = QPainter(self.label.pixmap())
        painter.setPen(QPen(Qt.red, 5))
        painter.drawPoint(x, y)
        painter.end()
        self.label.update()

        # Update the label with the current position
        self.position_label.setText(f"Current Position: {self.current_latitude:.6f}, {self.current_longitude:.6f}")

    def undo_last_action(self):
        if not self.action_history:
            return
        last_action, index = self.action_history.pop()
        if last_action == 'add':
            self.click_history.pop(index)
        self.redraw_points()

    def redraw_points(self):
        self.label.setPixmap(self.base_pixmap.copy())
        painter = QPainter(self.label.pixmap())
        painter.setPen(QPen(Qt.red, 5))

        for x, y, lat, lon in self.click_history:
            painter.drawPoint(x, y)

        painter.end()
        self.label.update()
        self.update_csv()

    def update_csv(self):
        with open(self.csv_file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            for _, _, lat, lon in self.click_history:
                writer.writerow([lat, lon])

    def ros_spin_once(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.1)

class ROSSubscriberNode(Node):
    def __init__(self, gui_app):
        super().__init__('global_position_subscriber')
        self.gui_app = gui_app
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert latitude and longitude to degrees and store in GUI application
        self.gui_app.current_latitude = msg.lat * 1e-7
        self.gui_app.current_longitude = msg.lon * 1e-7
        self.gui_app.update_current_position()

    def start_ros_node(self):
        print("ROS 2 Node initialized.")  # Corrected: Remove ROS init from here

def main():
    package_share_directory = get_package_share_directory('dashboard')  # Replace 'dashboard' with your package name
    api_path = os.path.join(package_share_directory, 'config', 'naver_api.txt')
    credentials = load_naver_api_credentials(api_path)
    CLIENT_ID = credentials['CLIENT_ID']
    CLIENT_SECRET = credentials['CLIENT_SECRET']
    size = f'{width}x{height}'
    image = fetch_map_image(latitude, longitude, zoom, maptype, size, CLIENT_ID, CLIENT_SECRET)
    csv_file_path = os.path.join(package_share_directory, 'output', 'clicked_coordinates.csv')
    
    # Initialize ROS 2
    rclpy.init(args=None)  # Corrected: Initialize ROS2 before QApplication

    app = QApplication(sys.argv)
    window = MapClickApp(image, latitude, longitude, zoom, csv_file_path)
    window.show()
    
    # Clean up ROS when exiting
    app.aboutToQuit.connect(lambda: rclpy.shutdown())
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
