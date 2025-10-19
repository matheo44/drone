from mpu6050 import mpu6050
import time
sensor = mpu6050(0x68)

while True: 

    accel_data = sensor.get_accel_data()

    gyro_data = sensor.get_gyro_data()

    x = round(accel_data["x"], 2)
    y = round(accel_data["y"], 2)
    z = round(accel_data["z"], 2)
    print("x ",x,"y ",y, "z ", z)

    
    x = round(gyro_data["x"], 2)
    y = round(gyro_data["y"], 2)
    z = round(gyro_data["z"], 2)
    print("gyro  x ",x, "y ", y, "z ", z)
    print("   ")

    time.sleep(1)
    
