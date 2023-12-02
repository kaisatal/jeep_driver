import smbus
import time

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
        print(f"Error reading AS5600 sensor: {e}")
        return None

if __name__ == "__main__":
    try:
        while True:
            # Read and print the angle from the AS5600 sensor
            angle = read_as5600_angle()
            if angle is not None:
                print(f"Current Angle: {angle:.2f} degrees")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program terminated by user.")
