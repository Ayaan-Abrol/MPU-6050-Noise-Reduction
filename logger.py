import serial
import csv
import time

# ==== CONFIG ====
SERIAL_PORT = 'COM3'      # Change if your Pico uses a different COM port
BAUD_RATE = 115200
LOG_FILE = "imu_data.csv"
# =================

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException:
    print(f"Could not open {SERIAL_PORT}. Make sure the Pico is connected and no other program is using it.")
    exit()

time.sleep(2)  # Wait for Pico to reset

# Open CSV file
with open(LOG_FILE, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["time", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])

    print("[*] Logging data... Press Ctrl+C to stop")

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            # Only process lines with 7 comma-separated values
            parts = line.split(',')
            if len(parts) != 7:
                continue

            try:
                t, ax, ay, az, gx, gy, gz = map(float, parts)
            except ValueError:
                continue  # skip lines that cannot convert to float

            # Print live data
            print(f"t={t:.2f}s | Acc=({ax:.2f},{ay:.2f},{az:.2f}) | Gyro=({gx:.2f},{gy:.2f},{gz:.2f})")

            # Write to CSV
            writer.writerow([t, ax, ay, az, gx, gy, gz])
            f.flush()

    except KeyboardInterrupt:
        print("\n[!] Logging stopped by user.")
        ser.close()
