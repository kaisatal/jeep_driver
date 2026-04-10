#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import smbus

# Define the I2C address of the AS5600 sensor
SENSOR_I2C_ADDRESS = 0x36

# Register addresses for the AS5600 sensor
ANGLE_MSB_REG = 0x0C
ANGLE_LSB_REG = 0x0D

# Create an smbus object for I2C communication
bus = smbus.SMBus(1)  # Use 1 for Jetson Nano

def read_as5600_angle():
    try:
        # Read MSB and LSB data from the sensor
        msb = bus.read_byte_data(SENSOR_I2C_ADDRESS, ANGLE_MSB_REG)
        lsb = bus.read_byte_data(SENSOR_I2C_ADDRESS, ANGLE_LSB_REG)

        # Combine MSB and LSB to get the 12-bit angle data
        angle_data = (msb << 8) | lsb
        # Convert 12-bit data to degrees (0 - 360)
        angle_degrees = (angle_data * 360) / 4096
        return angle_degrees

    except Exception as e:
        print(f"Error reading AS5600 sensor: {e}")
        return None

class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')

        self.pub = self.create_publisher(AckermannDrive, 'angle_feedback', 1) # Only the latest measurement is valid
        self.feedback = AckermannDrive()
        self.avg_filter = [0]*50
        self.prev_ang = -1
        self.timer = self.create_timer(0.02, self.timer_callback) # 50 Hz
    
    # Read and publish the angle from the AS5600 sensor
    def timer_callback(self):
        angle = read_as5600_angle()
        if angle is not None and angle < 360:
            self.avg_filter.pop(0)
            self.avg_filter.append(angle)
            avg_angle = round(174 - sum(self.avg_filter)/len(self.avg_filter)) # TODO: might remove the arbitrary range shift
            if avg_angle != self.prev_ang:
                self.feedback.steering_angle = float(avg_angle)
                self.prev_ang = avg_angle
            self.pub.publish(self.feedback)

def main():
    rclpy.init()
    node = FeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Feedback node.")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
