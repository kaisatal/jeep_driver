import rospy
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

        # Convert 12-bit data to degrees (0-360)
        angle_degrees = (angle_data * 360) / 4096

        return angle_degrees

    except Exception as e:
        rospy.ERROR(f"Error reading AS5600 sensor: {e}")
        return None

def main():
    rospy.init_node('feedback_node', anonymous=True)
    pub = rospy.Publisher('feedback', AckermannDrive, queue_size = 1)
    feedback = AckermannDrive()
    rate = rospy.Rate(100)

    avg_filter = [0]*200
    prev_ang = -1

    # Read and publish the angle from the AS5600 sensor
    while not rospy.is_shutdown():
        angle = read_as5600_angle()
        if angle is not None and angle<360:
            avg_filter.pop(0)
            avg_filter.append(angle)
            avg_angle = round(250-sum(avg_filter)/len(avg_filter))
            if avg_angle != prev_ang:
                feedback.steering_angle = avg_angle
                pub.publish(feedback)
                prev_ang = avg_angle
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass