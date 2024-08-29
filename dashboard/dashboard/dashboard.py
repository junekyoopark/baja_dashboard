import sys
from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QFrame
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import random
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, SensorCombined
from std_msgs.msg import Int32
import canusb

# Initialize global variables
lat = 0.0
lon = 0.0
lat_g = 0.0
lon_g = 0.0
lap_count = 0

class GGDiagramCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots()
        super(GGDiagramCanvas, self).__init__(self.fig)
        self.setParent(parent)
        
        # Set background to black and text/lines to white
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        self.ax.spines['top'].set_color('white')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['left'].set_color('white')
        self.ax.spines['right'].set_color('white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')

        self.ax.set_aspect('equal')  # Ensure the plot is square
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_xlabel('Lateral G')
        self.ax.set_ylabel('Longitudinal G')
        self.ax.set_title('G-G Diagram')
        self.history = []

    def plot(self, lateral_g, longitudinal_g):
        # Add new data to history
        self.history.append((lateral_g, longitudinal_g))

        # Clear the plot and reset the aspect ratio and limits
        self.ax.clear()
        
        # Set background and text colors again after clearing
        self.ax.set_facecolor('black')
        self.ax.spines['top'].set_color('white')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['left'].set_color('white')
        self.ax.spines['right'].set_color('white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')

        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_xlabel('Lateral G')
        self.ax.set_ylabel('Longitudinal G')
        self.ax.set_title('G-G Diagram')


        # Plot all previous points as white dots and connect them with lines
        if self.history:
            history_x, history_y = zip(*self.history)
            self.ax.scatter(history_x, history_y, color='white', s=10, zorder=2)  # Small white dots
            self.ax.plot(history_x, history_y, color='grey', linewidth=1, zorder=1)  # White line connecting points

        # Plot the current point in a much larger red dot
        self.ax.scatter(lateral_g, longitudinal_g, color='red', s=1000, zorder=3)  # Much larger red dot

        self.draw()

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()

        self.setWindowTitle("Vehicle Telemetry")
        
        # Set the background to black and text to white
        self.setStyleSheet("""
            QMainWindow {
                background-color: black;
            }
            QLabel {
                color: white;
            }
        """)

        # Create labels
        self.speed_value_label = QLabel("0")
        self.speed_unit_label = QLabel(" km/h")
        self.battery_value_label = QLabel("100")
        self.battery_unit_label = QLabel("%")
        self.lap_label = QLabel("HAVE A GREAT RACE")
        
        # Set text alignment for numeric labels
        self.speed_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.battery_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        # Set a fixed width for the numeric labels to prevent shifting
        self.speed_value_label.setFixedWidth(200)  # Adjust the width as needed
        self.battery_value_label.setFixedWidth(200)  # Adjust the width as needed

        # Create GG Diagram
        self.gg_canvas = GGDiagramCanvas(self)
        
        # Create a vertical line between labels
        between_labels_line = QFrame()
        between_labels_line.setFrameShape(QFrame.VLine)
        between_labels_line.setFrameShadow(QFrame.Sunken)
        between_labels_line.setMinimumWidth(2)  # Set minimum width for visibility
        
        # Create layout for the speed labels
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(self.speed_value_label)
        speed_layout.addWidget(self.speed_unit_label)
        
        # Create layout for the battery labels
        battery_layout = QHBoxLayout()
        battery_layout.addWidget(self.battery_value_label)
        battery_layout.addWidget(self.battery_unit_label)
        
        # Create layout for the labels (top row)
        top_layout = QHBoxLayout()
        top_layout.addLayout(speed_layout)
        top_layout.addSpacing(40)
        top_layout.addWidget(between_labels_line)  # Add vertical line between labels
        top_layout.addSpacing(40)
        top_layout.addLayout(battery_layout)

        # Create a wrapper layout to center the top_layout
        wrapper_layout = QVBoxLayout()
        wrapper_layout.addLayout(top_layout)
        wrapper_layout.setAlignment(Qt.AlignCenter)
        
        # Create a horizontal line
        horizontal_line = QFrame()
        horizontal_line.setFrameShape(QFrame.HLine)
        horizontal_line.setFrameShadow(QFrame.Sunken)
        horizontal_line.setMinimumHeight(2)  # Set minimum height for visibility

        # Create a layout for the right side (2/3 of the space)
        right_layout = QVBoxLayout()
        right_layout.addLayout(wrapper_layout)  # Add the wrapper layout, which centers the top_layout
        right_layout.addWidget(horizontal_line)  # Add horizontal line
        right_layout.addWidget(self.lap_label, alignment=Qt.AlignCenter)  # Center the lap label

        # Create a vertical line between G-G diagram and labels
        vertical_line = QFrame()
        vertical_line.setFrameShape(QFrame.VLine)
        vertical_line.setFrameShadow(QFrame.Sunken)
        vertical_line.setMinimumWidth(2)  # Set minimum width for visibility

        # Create main layout
        main_layout = QHBoxLayout()
        main_layout.addWidget(self.gg_canvas, 1)  # 1/3 space for g-g diagram
        main_layout.addWidget(vertical_line)      # Add vertical line
        main_layout.addLayout(right_layout, 2)    # 2/3 space for labels
        
        # Create a widget for the central area and set the main layout
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Start the CAN reading in a separate thread
        self.can_thread = threading.Thread(target=self.run_can_reader)
        self.can_thread.daemon = True
        self.can_thread.start()

        # Store ROS2 node and initialize the timer for spinning ROS
        self.node = node
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(1)  # Spin ROS2 at 10 Hz

    def run_can_reader(self):
        # Initialize CAN interface
        serial_port = "/dev/ttyCANUSB"
        baud_rate = 2500000
        ser = canusb.init_can_interface(serial_port, baud_rate)
        if ser is not None:
            canusb.read_can_frame(ser)
            ser.close()

    def spin_ros(self):
        """Spin ROS2 node periodically."""
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def resizeEvent(self, event):
        # Calculate the new font size based on the window dimensions
        new_font_size = self.width() // 20  # Adjust the divisor to change scaling
        font = QFont("Arial", new_font_size, QFont.Bold)

        # Apply the new font size to the labels
        self.speed_value_label.setFont(font)
        self.speed_unit_label.setFont(font)
        self.battery_value_label.setFont(font)
        self.battery_unit_label.setFont(font)
        self.lap_label.setFont(font)

        super().resizeEvent(event)

    def update_telemetry(self):
        """Update the telemetry data in the UI."""
        global lat_g, lon_g, lap_count
        
        # Update the g-g diagram
        self.gg_canvas.plot(lat_g, lon_g)
        lap = lap_count

        with canusb.var_lock:
            speed = canusb.speed_rpm
            battery = canusb.bus_voltage

        # Update labels
        self.speed_value_label.setText(f"{speed:.2f}")
        self.battery_value_label.setText(f"{battery}")
        self.lap_label.setText(f"LAP {lap}")

        

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.lat_lon_callback, qos_profile)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.g_g_callback, qos_profile)
        self.create_subscription(Int32, '/lap_count', self.lap_count_callback, qos_profile)

    def lat_lon_callback(self, msg):
        global lat 
        global lon
        lat = msg.lat
        lon = msg.lon
        self.get_logger().info('Current Latitude: %.6f' % msg.lat)

    def g_g_callback(self, msg):
        global lat_g
        global lon_g
        lat_g = msg.accelerometer_m_s2[1]
        lon_g = msg.accelerometer_m_s2[0]
        self.get_logger().info('Current lat_g: %.6f' % msg.accelerometer_m_s2[1])

    def lap_count_callback(self, msg):
        global lap_count
        lap_count = msg.data
        self.get_logger().info('Current lap: %d' % msg.data)

def main(args=None):
    # Initialize ROS2 only once
    rclpy.init(args=args)

    # Create ROS2 node
    telemetry_node = TelemetryNode()

    # Initialize the Qt application
    app = QApplication(sys.argv)

    # Pass the ROS2 node to the MainWindow
    window = MainWindow(telemetry_node)
    window.show()

    # Start the Qt event loop
    sys.exit(app.exec_())

    # Properly shutdown ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
