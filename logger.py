import serial
import csv
import time

SERIAL_PORT = 'COM3'     
BAUD_RATE = 115200
LOG_FILE = "imu_data.csv"
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException:
    print(f"ERROR")
    exit()

time.sleep(2)  


with open(LOG_FILE, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["time", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])

    print("Logging data")

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 7:
                continue

            try:
                t, ax, ay, az, gx, gy, gz = map(float, parts)
            except ValueError:
                continue  

           
            print(f"t={t:.2f}s | Acc=({ax:.2f},{ay:.2f},{az:.2f}) | Gyro=({gx:.2f},{gy:.2f},{gz:.2f})")

          
            writer.writerow([t, ax, ay, az, gx, gy, gz])
            f.flush()

    except KeyboardInterrupt:
        print("Logging stopped")
        ser.close()
