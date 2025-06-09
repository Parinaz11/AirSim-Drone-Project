import airsim
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from datetime import datetime

output_dir = r"C:\\Users\\parin\Desktop\\IMU_Data_Collection"
# os.makedirs(output_dir, exist_ok=True)

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Drone Take off
client.takeoffAsync().join()

# Defining square path parameters (10m x 10m square at fixed altitude)
side_length = 10  # meters
speed = 5  # m/s
z = -5  # Fixed altitude (negative in NED coordinates)
corners = [
    (0, 0, z),          # Corner 1
    (side_length, 0, z), # Corner 2
    (side_length, side_length, z), # Corner 3
    (0, side_length, z)  # Corner 4
]
duration_per_side = side_length / speed  # Time to traverse one side
loops = 1  # Number of square loops

# Lists to store IMU data and timestamps
timestamps = []
accel_x, accel_y, accel_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []

# Move in a square path and collect IMU data
start_time = time.time()
for _ in range(loops):
    for corner in corners:
        # Move to each corner
        client.moveToPositionAsync(corner[0], corner[1], corner[2], speed).join()

        # Collectting IMU data for the duration of one side
        side_start = time.time()
        while time.time() - side_start < duration_per_side:
            # Get IMU data
            imu_data = client.getImuData()
            
            # Storing timestamp (convert nanoseconds to seconds)
            timestamps.append(imu_data.time_stamp / 1e9)
            
            # Storing linear acceleration
            accel_x.append(imu_data.linear_acceleration.x_val)
            accel_y.append(imu_data.linear_acceleration.y_val)
            accel_z.append(imu_data.linear_acceleration.z_val)
            
            # Storing angular velocity
            gyro_x.append(imu_data.angular_velocity.x_val)
            gyro_y.append(imu_data.angular_velocity.y_val)
            gyro_z.append(imu_data.angular_velocity.z_val)

            # Small delay to avoid overwhelming the simulator
            time.sleep(0.1)

# Landing the drone
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)

# Saving IMU data to CSV
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
data = {
    'Timestamp (s)': timestamps,
    'Accel X (m/s²)': accel_x,
    'Accel Y (m/s²)': accel_y,
    'Accel Z (m/s²)': accel_z,
    'Gyro X (rad/s)': gyro_x,
    'Gyro Y (rad/s)': gyro_y,
    'Gyro Z (rad/s)': gyro_z
}
df = pd.DataFrame(data)
csv_path = os.path.join(output_dir, f'imu_data_{timestamp}.csv')
df.to_csv(csv_path, index=False)
print("File saved to", csv_path)

plt.figure(figsize=(10, 10))

# Subplot 1: Linear acceleration
plt.subplot(2, 1, 1)
plt.plot(timestamps, accel_x, label='Accel X')
plt.plot(timestamps, accel_y, label='Accel Y')
plt.plot(timestamps, accel_z, label='Accel Z')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.title('IMU Linear Acceleration vs Time')
plt.legend()
plt.grid(True)

# Subplot 2: Angular velocity
plt.subplot(2, 1, 2)
plt.plot(timestamps, gyro_x, label='Gyro X')
plt.plot(timestamps, gyro_y, label='Gyro Y')
plt.plot(timestamps, gyro_z, label='Gyro Z')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('IMU Angular Velocity vs Time')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(output_dir, f'plots_{timestamp}.png'))
plt.close()
print("Saved plots at", f'plots_{timestamp}.png')