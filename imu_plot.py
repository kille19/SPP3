import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

CSV_PATH = "imu_fusion_output.csv"

def quat_to_rpy(w, x, y, z):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # pitch (y-axis rotation)
    sinp = np.sqrt(1 + 2 * (w * y - x * z))
    cosp = np.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2
    
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw
    
def main():
    df = pd.read_csv(CSV_PATH, sep=";")

    time = df['t'].to_numpy()
    
    # plot 1: Accelerometer data
    plt.figure()
    plt.plot(time, df['acc0_norm'].to_numpy(), label='acc0_norm')
    plt.plot(time, df['acc1_norm'].to_numpy(), label='acc1_norm')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration Norm [m/s²]')
    plt.grid(True)
    plt.legend()
    
    # plot 2: Gyroscope data
    plt.figure()
    plt.plot(time, df['gyro0_norm'].to_numpy(), label='gyro0_norm')
    plt.plot(time, df['gyro1_norm'].to_numpy(), label='gyro1_norm')
    plt.xlabel('Time [s]')
    plt.ylabel('Gyroscope Norm [rad/s]')
    plt.grid(True)
    plt.legend()
    
    # plot 3: Q_relative norm
    qw = df['qrel_w'].to_numpy()
    qx = df['qrel_x'].to_numpy()
    qy = df['qrel_y'].to_numpy()
    qz = df['qrel_z'].to_numpy()
    qrel_norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    
    plt.figure()
    plt.plot(time, qrel_norm, label='Q_relative Norm')
    plt.xlabel('Time [s]')
    plt.ylabel('Quaternion Norm')
    plt.grid(True)
    plt.legend()
    
    # plot 4: Roll, Pitch, Yaw 
    roll, pitch, yaw = quat_to_rpy(qw, qx, qy, qz)
    roll = np.unwrap(roll) * (180.0 / np.pi)
    pitch = np.unwrap(pitch) * (180.0 / np.pi)
    yaw = np.unwrap(yaw) * (180.0 / np.pi)
    
    plt.figure()
    plt.plot(time, roll, label='Roll [deg]')
    plt.plot(time, pitch, label='Pitch [deg]')
    plt.plot(time, yaw, label='Yaw [deg]')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [degrees]')
    plt.grid(True)
    plt.legend()
    
    # plot 5
    plt.figure()
    plt.plot(time, df["err_angle"].to_numpy())
    plt.xlabel('Time [s]')
    plt.ylabel('Error Angle [deg]')
    plt.grid(True)
    plt.legend()
    
    
    plt.show()
    
    
if __name__ == "__main__":
    main()