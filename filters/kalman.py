import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



class KalmanFilter1D:
    """
    Kalman Filter
    """
    def __init__(self, Q_angle=1e-3, Q_bias=3e-3, R_measure=1e-1):
        self.Q_angle = float(Q_angle)
        self.Q_bias = float(Q_bias)
        self.R_measure = float(R_measure)

        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0

    
        self.P = [[0.0, 0.0],
                  [0.0, 0.0]]

    def update(self, z_angle_deg: float, gyro_dps: float, dt: float) -> float:
  
        self.rate = gyro_dps - self.bias
        self.angle += dt * self.rate

       
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        y = z_angle_deg - self.angle          
        self.angle += K0 * y
        self.bias  += K1 * y

      
        P00, P01 = self.P[0][0], self.P[0][1]
        self.P[0][0] -= K0 * P00
        self.P[0][1] -= K0 * P01
        self.P[1][0] -= K1 * P00
        self.P[1][1] -= K1 * P01

        return self.angle


def accel_to_angles_deg(ax, ay, az):
   
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan(-ax / max(1e-9, math.sqrt(ay*ay + az*az))))
    return roll, pitch

def normalize_time_seconds(t_raw: np.ndarray) -> np.ndarray:
    
    t = t_raw.astype(float)
    if t.max() > 1e5:         
        return t / 1000.0
    dt = np.diff(t)
    med_dt = np.median(dt[dt > 0]) if np.any(dt > 0) else 0.01
    return t/1000.0 if med_dt > 1.5 else t

def autoscale_gyro_to_dps(g: np.ndarray) -> (np.ndarray, str):
    
    x = np.asarray(g, dtype=float)
    q99 = np.percentile(np.abs(x), 99)

    # raw LSB 
    if q99 > 500.0:
        best = x
        label = "deg/s (assumed)"
        sens = [131.0, 65.5, 32.8, 16.4]  # ±250/500/1000/2000 dps
        candidates = [(x/s, f"raw/LSB ÷ {s}") for s in sens]
        scored = []
        for arr, lab in candidates:
            q = np.percentile(np.abs(arr), 99)
            score = 1.0/(abs(q-100)+1e-6) if 20 <= q <= 500 else 0.0
            scored.append((score, arr, lab))
        score, best, label = max(scored, key=lambda z: z[0])
        return best, label

    
    if 3.0 <= q99 <= 20.0:
        return np.degrees(x), "rad/s → deg/s"

    return x, "deg/s (assumed)"

def estimate_initial_bias(gyro_dps: np.ndarray, t_sec: np.ndarray, window_s: float = 1.0) -> float:
    
    t0 = float(t_sec[0])
    mask = (t_sec - t0) <= window_s
    return float(np.mean(gyro_dps[mask])) if np.any(mask) else 0.0

def run_kalman(filename="mpu_data.csv",
               remove_gyro_bias=True,
               Q_ANGLE=1e-3, Q_BIAS=3e-3, R_MEASURE=1e-1):

    
    df = pd.read_csv(filename)
    df.columns = df.columns.str.strip().str.lower()
    required = ["time", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}\nFound: {list(df.columns)}")

    t = normalize_time_seconds(df["time"].to_numpy())
    df["time"] = t

    
    gx_dps, gx_label = autoscale_gyro_to_dps(df["gyro_x"].to_numpy())
    gy_dps, gy_label = autoscale_gyro_to_dps(df["gyro_y"].to_numpy())
    gz_dps, gz_label = autoscale_gyro_to_dps(df["gyro_z"].to_numpy())  

    
    bx = by = 0.0
    if remove_gyro_bias:
        bx = estimate_initial_bias(gx_dps, t, 1.0)
        by = estimate_initial_bias(gy_dps, t, 1.0)
        gx_dps = gx_dps - bx
        gy_dps = gy_dps - by

    
    ax = df["accel_x"].to_numpy(dtype=float)
    ay = df["accel_y"].to_numpy(dtype=float)
    az = df["accel_z"].to_numpy(dtype=float)

    
    kf_roll  = KalmanFilter1D(Q_angle=Q_ANGLE, Q_bias=Q_BIAS, R_measure=R_MEASURE)
    kf_pitch = KalmanFilter1D(Q_angle=Q_ANGLE, Q_bias=Q_BIAS, R_measure=R_MEASURE)

    roll_raw, pitch_raw, roll_kal, pitch_kal = [], [], [], []
    prev_t = t[0]

    for i in range(len(df)):
        dt = max(float(t[i] - prev_t), 1e-4)
        prev_t = t[i]

        roll_acc, pitch_acc = accel_to_angles_deg(ax[i], ay[i], az[i])
        roll_est  = kf_roll.update(roll_acc,  gx_dps[i], dt)
        pitch_est = kf_pitch.update(pitch_acc, gy_dps[i], dt)

        roll_raw.append(roll_acc)
        pitch_raw.append(pitch_acc)
        roll_kal.append(roll_est)
        pitch_kal.append(pitch_est)

    
    out = df.copy()
    out["roll_raw"] = roll_raw
    out["pitch_raw"] = pitch_raw
    out["roll_filtered"] = roll_kal
    out["pitch_filtered"] = pitch_kal
    out.to_csv("mpu_data_kalman_output.csv", index=False)

    
    print("=== Scaling / Bias Summary ===")
    print(f"time units: seconds (auto-normalized)")
    print(f"gyro_x: {gx_label}; bias removed: {bx:.4f} dps")
    print(f"gyro_y: {gy_label}; bias removed: {by:.4f} dps")
    print("Saved → mpu_data_kalman_output.csv")

    
    plt.figure(figsize=(10, 6))
    plt.plot(t, roll_raw,  label="Raw Roll (Accel)",  alpha=0.55)
    plt.plot(t, roll_kal,  label="Kalman Roll")
    plt.plot(t, pitch_raw, label="Raw Pitch (Accel)", alpha=0.55)
    plt.plot(t, pitch_kal, label="Kalman Pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.title("Sensor Fusion (Kalman)")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    
    run_kalman("mpu_data.csv",
               remove_gyro_bias=True,
               Q_ANGLE=1e-3, Q_BIAS=3e-3, R_MEASURE=1e-1)
