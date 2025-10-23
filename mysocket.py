import socket
import threading
import time

from mpu6050 import mpu6050
import math
from my_compas import compas
from my_compas import KalmanFilter


#### initialisation mpu6050 ####
#initialisation du capteur et des filtres

mpu = mpu6050(0x68)
kalman_pitch = KalmanFilter()
kalman_roll = KalmanFilter()
dt = 0.01
time.sleep(1)

#calibration du mpu

accel = mpu.get_accel_data()
pitch = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
roll = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
kalman_pitch.angle = pitch
kalman_roll.angle = roll

#### début de la partie socket ####
HOST = '192.168.1.54'  # IP du Raspberry Pi
PORT = 65432


def encode_data(mpu_data, engine_state): # retourne une string data_encoded
	
    data = mpu_data+";"+';'.join(engine_state)
    return data

def decode(data): # parametre : string data_encoded
    data = data.split(";")
    print(f'trust: {data[0]} roll: {data[1]} pitch: {data[2]}')
     


def receive_data(s):
    while True:
        data = s.recv(1024).decode('utf-8')
        decode(data)
        if not data:
            break
        #print(f"Reçu du PC: {data}")

def send_data(s):
    while True:
        engine_state = ["10","10","20","20"]
        mpu_data = compas(kalman_pitch, kalman_roll, mpu, dt )
        data_rpi = encode_data(mpu_data, engine_state)

        s.sendall(data_rpi.encode('utf-8'))
        time.sleep(0.2)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    
    # Thread pour recevoir les données
    receive_thread = threading.Thread(target=receive_data, args=(s,))
    receive_thread.start()
    
    # Thread pour envoyer des données
    send_thread = threading.Thread(target=send_data, args=(s,))
    send_thread.start()
    
    receive_thread.join()
    send_thread.join()


