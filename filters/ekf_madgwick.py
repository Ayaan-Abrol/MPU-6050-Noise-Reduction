import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def accel_to_angles(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan(-ax / max(1e-9, math.sqrt(ay*ay + az*az))))
    return roll, pitch

def quat_to_euler(q0, q1, q2, q3):
    sinr = 2.0 * (q0*q1 + q2*q3)
    cosr = 1.0 - 2.0 * (q1*q1 + q2*q2)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (q0*q2 - q3*q1)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi/2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (q0*q3 + q1*q2)
    cosy = 1.0 - 2.0 * (q2*q2 + q3*q3)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw

def run_ekf(ax, ay, az, gx, gy, t, q_angle=1e-3, q_bias=3e-3, r_angle=1e-1):
    n = len(ax)
    x = np.zeros(4)
    P = np.eye(4) * 0.1
    Q = np.diag([q_angle, q_angle, q_bias, q_bias])
    R = np.diag([r_angle, r_angle])
    roll_out = np.zeros(n)
    pitch_out = np.zeros(n)
    t_prev = t[0]
    for i in range(n):
        dt = max(float(t[i] - t_prev), 1e-4)
        t_prev = t[i]
        roll, pitch, bx, by = x
        roll_p = roll + dt * (gx[i] - bx)
        pitch_p = pitch + dt * (gy[i] - by)
        x_p = np.array([roll_p, pitch_p, bx, by])
        F = np.array([
            [1.0, 0.0, -dt, 0.0],
            [0.0, 1.0, 0.0, -dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        P = F @ P @ F.T + Q
        x = x_p
        rz, pz = accel_to_angles(ax[i], ay[i], az[i])
        z = np.array([rz, pz])
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ y
        P = (np.eye(4) - K @ H) @ P
        roll_out[i] = x[0]
        pitch_out[i] = x[1]
    return roll_out, pitch_out

def madgwick_step(q, gx, gy, gz, ax, ay, az, beta, dt):
    q1, q2, q3, q4 = q
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return q
    ax /= norm
    ay /= norm
    az /= norm
    f1 = 2*(q2*q4 - q1*q3) - ax
    f2 = 2*(q1*q2 + q3*q4) - ay
    f3 = 2*(0.5 - q2*q2 - q3*q3) - az
    J_11or24 = 2*q3
    J_12or23 = 2*q4
    J_13or22 = 2*q1
    J_14or21 = 2*q2
    J_32 = 2*J_14or21
    J_33 = 2*J_11or24
    g1 = J_14or21*f2 - J_11or24*f1
    g2 = J_12or23*f1 + J_13or22*f2 - J_32*f3
    g3 = J_12or23*f2 - J_33*f3 - J_13or22*f1
    g4 = J_14or21*f1 + J_11or24*f2
    norm_g = math.sqrt(g1*g1 + g2*g2 + g3*g3 + g4*g4)
    if norm_g != 0:
        g1 /= norm_g
        g2 /= norm_g
        g3 /= norm_g
        g4 /= norm_g
    qDot1 = 0.5 * (-q2*gx - q3*gy - q4*gz) - beta*g1
    qDot2 = 0.5 * (q1*gx + q3*gz - q4*gy) - beta*g2
    qDot3 = 0.5 * (q1*gy - q2*gz + q4*gx) - beta*g3
    qDot4 = 0.5 * (q1*gz + q2*gy - q3*gx) - beta*g4
    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt
    q4 += qDot4 * dt
    norm_q = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
    q1 /= norm_q
    q2 /= norm_q
    q3 /= norm_q
    q4 /= norm_q
    return np.array([q1, q2, q3, q4])

def run_madgwick(ax, ay, az, gx, gy, gz, t, beta=0.1):
    n = len(ax)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    roll = np.zeros(n)
    pitch = np.zeros(n)
    t_prev = t[0]
    for i in range(n):
        dt = max(float(t[i] - t_prev), 1e-4)
        t_prev = t[i]
        gx_r = math.radians(gx[i])
        gy_r = math.radians(gy[i])
        gz_r = math.radians(gz[i])
        q = madgwick_step(q, gx_r, gy_r, gz_r, ax[i], ay[i], az[i], beta, dt)
        r, p, _ = quat_to_euler(*q)
        roll[i] = r
        pitch[i] = p
    return roll, pitch

def main(alpha=0.5):
    df = pd.read_csv("mpu_data.csv")
    df.columns = df.columns.str.strip().str.lower()
    t = df["time"].to_numpy(dtype=float)
    ax = df["accel_x"].to_numpy(dtype=float)
    ay = df["accel_y"].to_numpy(dtype=float)
    az = df["accel_z"].to_numpy(dtype=float)
    gx = df["gyro_x"].to_numpy(dtype=float)
    gy = df["gyro_y"].to_numpy(dtype=float)
    gz = df["gyro_z"].to_numpy(dtype=float)
    n = len(df)
    roll_acc = np.zeros(n)
    pitch_acc = np.zeros(n)
    for i in range(n):
        roll_acc[i], pitch_acc[i] = accel_to_angles(ax[i], ay[i], az[i])
    roll_ekf, pitch_ekf = run_ekf(ax, ay, az, gx, gy, t)
    roll_mad, pitch_mad = run_madgwick(ax, ay, az, gx, gy, gz, t, beta=0.1)
    roll_fused = alpha * roll_ekf + (1.0 - alpha) * roll_mad
    pitch_fused = alpha * pitch_ekf + (1.0 - alpha) * pitch_mad
    out = df.copy()
    out["roll_accel"] = roll_acc
    out["pitch_accel"] = pitch_acc
    out["roll_ekf"] = roll_ekf
    out["pitch_ekf"] = pitch_ekf
    out["roll_madgwick"] = roll_mad
    out["pitch_madgwick"] = pitch_mad
    out["roll_fused"] = roll_fused
    out["pitch_fused"] = pitch_fused
    out.to_csv("mpu_data_fused.csv", index=False)

    plt.figure(figsize=(11, 7))
    plt.subplot(2, 1, 1)
    plt.title("ROLL — accel / EKF / Madgwick / fused")
    plt.plot(t, roll_acc, label="accel", alpha=0.3)
    plt.plot(t, roll_ekf, label="ekf", alpha=0.7)
    plt.plot(t, roll_mad, label="madgwick", alpha=0.7)
    plt.plot(t, roll_fused, label="fused", linewidth=2)
    plt.ylabel("deg")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.title("PITCH — accel / EKF / Madgwick / fused")
    plt.plot(t, pitch_acc, label="accel", alpha=0.3)
    plt.plot(t, pitch_ekf, label="ekf", alpha=0.7)
    plt.plot(t, pitch_mad, label="madgwick", alpha=0.7)
    plt.plot(t, pitch_fused, label="fused", linewidth=2)
    plt.xlabel("time (s)")
    plt.ylabel("deg")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main(alpha=0.5)
