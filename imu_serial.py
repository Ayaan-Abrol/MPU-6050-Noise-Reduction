from machine import Pin, I2C
import time


class MPU6050:
    ADDRESS = 0x68

    def __init__(self, i2c):
        self.i2c = i2c
        
        self.i2c.writeto_mem(self.ADDRESS, 0x6B, b'\x00')
        self.gyro_bias = [0, 0, 0]

    def read_raw(self, reg):
        high = self.i2c.readfrom_mem(self.ADDRESS, reg, 1)[0]
        low = self.i2c.readfrom_mem(self.ADDRESS, reg + 1, 1)[0]
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def get_accel(self):
        ax = self.read_raw(0x3B) / 16384.0
        ay = self.read_raw(0x3D) / 16384.0
        az = self.read_raw(0x3F) / 16384.0
        return ax, ay, az

    def get_gyro(self):
        gx = self.read_raw(0x43) / 131.0 - self.gyro_bias[0]
        gy = self.read_raw(0x45) / 131.0 - self.gyro_bias[1]
        gz = self.read_raw(0x47) / 131.0 - self.gyro_bias[2]
        return gx, gy, gz

    def calibrate_gyro(self, samples=200):
        print("Calibrating gyro... Keep the MPU6050 still.")
        gx_sum = gy_sum = gz_sum = 0
        for _ in range(samples):
            gx_sum += self.read_raw(0x43) / 131.0
            gy_sum += self.read_raw(0x45) / 131.0
            gz_sum += self.read_raw(0x47) / 131.0
            time.sleep(0.01)
        self.gyro_bias = [gx_sum / samples, gy_sum / samples, gz_sum / samples]
        print("Calibration done. Bias:", self.gyro_bias)


i2c = I2C(0, scl=Pin(1), sda=Pin(0))
mpu = MPU6050(i2c)


mpu.calibrate_gyro()


while True:
    t = time.ticks_ms() / 1000
    ax, ay, az = mpu.get_accel()
    gx, gy, gz = mpu.get_gyro()

    
    print(f"{t:.3f},{ax:.3f},{ay:.3f},{az:.3f},{gx:.3f},{gy:.3f},{gz:.3f}")

    time.sleep(0.05)
