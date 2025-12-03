MPU-6050 Noise Reduction & Sensor Fusion

This branch contains the full IMU filtering pipeline for the MPU-6050, including:

Raw calibration data processing

Kalman angle filtering

EKF orientation estimation

Madgwick quaternion filtering

Fused EKF + Madgwick output

A Jupyter Notebook comparing all filters

The goal of this project is to reduce MPU-6050 noise and evaluate which filtering method provides the best roll/pitch estimation.

Data Format

All filters expect:

mpu_data.csv

with columns:

time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z


time in seconds

Accelerometer in g (scaled)

Gyro in deg/sec

Files in This Branch
1. kalman.py

Runs a 2-axis Kalman filter on roll and pitch.

Outputs:

mpu_data_kalman_output.csv
  roll_raw
  pitch_raw
  roll_filtered
  pitch_filtered
  
fusion_ekf_madgwick_fused.py

Runs:

EKF on roll/pitch + gyro bias

Madgwick quaternion filter

Fused EKF + Madgwick output

Outputs:

mpu_data_fused.csv
  roll_accel
  pitch_accel
  roll_ekf
  pitch_ekf
  roll_madgwick
  pitch_madgwick
  roll_fused
  pitch_fused

3. filter_comparison.ipynb

Jupyter notebook comparing:

Kalman

EKF

Madgwick

Fused

Includes:

Overlay plots for roll & pitch

MSE between all filter outputs

Interpretation of results

Filters Implemented
Kalman Filter

State: angle + gyro bias

Predict via gyro

Correct via accelerometer tilt

Good for: stable roll/pitch estimation.

Extended Kalman Filter (EKF)

State: [roll, pitch, bias_x, bias_y]

Models cross-axis coupling

Strong alignment with Kalman on roll

Good for: roll accuracy and bias handling.

Madgwick Filter

Quaternion-based

Uses gradient descent + gyro integration

Smoothest output

Good for: dynamic motion, smooth pitch response.

Fused Filter (EKF + Madgwick)

Late fusion:

roll_fused  = α·roll_ekf  + (1−α)·roll_madgwick
pitch_fused = α·pitch_ekf + (1−α)·pitch_madgwick


Best overall trade-off between stability, smoothness, and accuracy.

How to Run
Install dependencies:
pip install numpy pandas matplotlib

Run Kalman filter:
python kalman.py

Run EKF + Madgwick + Fused:
python fusion_ekf_madgwick_fused.py

Run comparison notebook:
jupyter notebook


Open:

filter_comparison.ipynb

Performance Summary

From the MSE comparison:

Roll
Best = EKF  
2nd = Fused  
Worst = Madgwick

Pitch
Best = Fused  
2nd = Madgwick  
Worst = EKF

Overall Best Filter: Fused (EKF + Madgwick)

Balancing accuracy, stability, and smoothness.


