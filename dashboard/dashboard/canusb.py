# canusb.py

import serial
import struct
import time
import threading

# Define constants for CAN frame IDs
CAN_FRAME_ID_MOTOR_SPEED = 0x180117EF

# Global variable for speed
speed_rpm = 0.0

# Lock for thread-safe access to speed_rpm
var_lock = threading.Lock()

def init_can_interface(serial_port, baud_rate):
    """
    Initialize CAN interface via serial port using pyserial.
    """
    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01,  # Small timeout to read data frequently without blocking
            write_timeout=0.01,
        )
        ser.flush()  # Flush any data already in the buffer
        return ser
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        return None

def read_can_frame(ser):
    """
    Read and interpret all data from a CAN frame.
    """
    global bus_voltage
    global bus_current
    global phase_current
    global speed_rpm

    FRAME_SIZE = 30
    while True:
        try:
            # Read exactly one frame size of data if available
            if ser.in_waiting >= FRAME_SIZE:
                frame_data = ser.read(FRAME_SIZE)
                arbitration_id, dlc = struct.unpack('<IB', frame_data[:5])
                print(frame_data)
                
                # Update variables with a lock
                with var_lock:
                    bus_voltage = (struct.unpack('<h', frame_data[6:8])[0]) * 0.1
                    bus_current = ((struct.unpack('<h', frame_data[8:10])[0]) * 0.1) - 3200.0
                    phase_current = ((struct.unpack('<h', frame_data[10:12])[0]) * 0.1) - 3200.0
                    speed_rpm = ((struct.unpack('<h', frame_data[12:14])[0]) * 0.1) - 3200.0

                    
                
                print("Interpreted Data:")
                print(f"Timestamp: {time.time():.3f}")
                print(f"Bus Voltage: {bus_voltage:.2f} V")
                print(f"Bus Current: {bus_current:.2f} A")
                print(f"Phase Current: {phase_current:.2f} A")
                print(f"Speed: {speed_rpm:.2f} RPM")
                print(f"Buffer read: {len(frame_data)} bytes")  # Debug: Print read length

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break
        except struct.error as e:
            print(f"Struct error: {e}")
            break

def main():
    serial_port = "/dev/ttyCANUSB"
    baud_rate = 2500000  # CAN bus speed

    ser = init_can_interface(serial_port, baud_rate)
    if ser is None:
        return

    read_can_frame(ser)

    ser.close()

if __name__ == "__main__":
    main()
