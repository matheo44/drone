from mpu6050 import mpu6050
import math
import time

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure

        self.angle = 0.0  # Angle estimé
        self.bias = 0.0   # Biais du gyroscope
        self.P = [[0.0, 0.0], [0.0, 0.0]]  # Matrice de covariance

    def get_angle(self, new_angle, new_rate, dt):
        # Prédiction
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Mise à jour de l'erreur de covariance
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Innovation
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

# Initialisation capteur et filtres
#mpu = mpu6050(0x68)
#kalman_pitch = KalmanFilter()
#kalman_roll = KalmanFilter()

#dt = 0.01  # 10 ms
#time.sleep(1)

# Calibration initiale
#accel = mpu.get_accel_data()
#pitch = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
#roll = math.degrees(math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)))
#kalman_pitch.angle = pitch
#kalman_roll.angle = roll

def compas(kalman_pitch, kalman_roll, mpu, dt):
    #while True:
       accel = mpu.get_accel_data()
       gyro = mpu.get_gyro_data()

       # Angles à partir de l'accéléromètre
       pitch_acc = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
       roll_acc = math.degrees(math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)))
       print("roll_acc: ", roll_acc, " pitch_acc: ", pitch_acc)
       # Angles avec filtre de Kalman
       pitch = kalman_pitch.get_angle(pitch_acc, gyro['y'], dt)
       roll = kalman_roll.get_angle(roll_acc, -gyro['x'], dt)

       print(f"Pitch: {pitch:.2f}° | Roll: {roll:.2f}°")
       time.sleep(dt)
       return(str( round(pitch_acc, 2) ) + "," + str( round( roll_acc, 2 ) ))
