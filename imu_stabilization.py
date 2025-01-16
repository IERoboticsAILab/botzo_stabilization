import smbus  # For IMU communication
import numpy as np
import time  # For delay
import socket  # For sending data in real-time

# Initialize I2C bus and MPU-6050 address
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
MPU_ADDRESS = 0x68  # Default I2C address for MPU-6050

# MPU-6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize MPU-6050
def initialize_mpu():
    bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0)  # Wake up the MPU-6050

def read_raw_data(addr):
    # Read two bytes of data from the given register
    high = bus.read_byte_data(MPU_ADDRESS, addr)
    low = bus.read_byte_data(MPU_ADDRESS, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def read_imu_data():
    # Read accelerometer and gyroscope data
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_XOUT_H + 2)
    accel_z = read_raw_data(ACCEL_XOUT_H + 4)

    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_XOUT_H + 2)
    gyro_z = read_raw_data(GYRO_XOUT_H + 4)

    # Convert to degrees/sec and g (scaled values depend on sensor settings)
    accel_scale = 16384.0  # Assuming +/- 2g range
    gyro_scale = 131.0  # Assuming +/- 250 degrees/sec range

    accel_x /= accel_scale
    accel_y /= accel_scale
    accel_z /= accel_scale

    gyro_x /= gyro_scale
    gyro_y /= gyro_scale
    gyro_z /= gyro_scale

    # Calculate pitch, roll, yaw (simplified, not accounting for fusion/filtering)
    pitch = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2)) * (180 / np.pi)
    roll = np.arctan2(-accel_x, accel_z) * (180 / np.pi)
    yaw = gyro_z  # Simplified yaw, typically requires integration

    return pitch, roll, yaw

def compute_rotation_matrix(pitch, roll, yaw):
    # Define rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(np.radians(roll)), -np.sin(np.radians(roll))],
                    [0, np.sin(np.radians(roll)), np.cos(np.radians(roll))]])
    R_y = np.array([[np.cos(np.radians(pitch)), 0, np.sin(np.radians(pitch))],
                    [0, 1, 0],
                    [-np.sin(np.radians(pitch)), 0, np.cos(np.radians(pitch))]])
    R_z = np.array([[np.cos(np.radians(yaw)), -np.sin(np.radians(yaw)), 0],
                    [np.sin(np.radians(yaw)), np.cos(np.radians(yaw)), 0],
                    [0, 0, 1]])
    return np.dot(R_z, np.dot(R_y, R_x))

def stabilize_robot(rotation_matrix):
    # Placeholder logic for stabilizing the robot
    # Use rotation_matrix to compute actuator adjustments
    print("Stabilizing robot with rotation matrix:")
    print(rotation_matrix)

def send_data_via_socket(data):
    # Send data to a server or another device using a socket
    host = '192.168.1.100'  # Replace with your server IP
    port = 5000  # Replace with your desired port

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        message = ','.join(map(str, data))
        s.sendto(message.encode(), (host, port))

# Main loop
initialize_mpu()
while True:
    imu_data = read_imu_data()
    pitch, roll, yaw = imu_data  # Parse angles
    rotation_matrix = compute_rotation_matrix(pitch, roll, yaw)
    stabilize_robot(rotation_matrix)
    send_data_via_socket([pitch, roll, yaw])
    time.sleep(0.1)  # Add delay for stability
