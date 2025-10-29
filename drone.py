import RPi.GPIO as GPIO
import time

# import des librairie du mpu 
from mpu6050 import mpu6050
import math
from my_compas import compas
from my_compas import KalmanFilter

#initialisation de la fonction des controle des moteurs
def engine_ctrl(pwm, trust):
    """Définit la vitesse du moteur (0% à 100%)"""
    # Convertir le pourcentage en cycle de duty (typiquement 5-10% pour 50Hz)
    duty_cycle = 5 + ( trust / 100.0) * 5
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Vitesse définie à {trust}% (Duty Cycle: {duty_cycle}%)")

# définition de la fonction de calcul des vitesse moteur requis pour la stabiliter pour un angle de rotation(algo PID)

def PID_ctrl(current_angle, angle_target, error_sum, error_prev ):

    #constantes des gains à regler
    Kp = 2.0
    Ki = 0.5
    Kd = 1.0
    
    #calcule le nombre de degrés à compenser
    error = angle_target - current_angle 
    #acculmulation du temps que met le drone à atteindre son objectif multiplier par le temps entre les corrections
    error_sum += error * 0.1 # 10 ms entre chaque correction (delta time)
    #dérivation de de l'érreur par dt
    derivative = (error - error_prev) / 0.1
    #appliacation des donnés avec les gains
    output = Kp * error + Ki * error_sum + Kd * derivative
    error_prev = error/2
    return output

## seting des pin en mode BCM
GPIO.setmode(GPIO.BCM)

# Fréquence PWM (typiquement 50Hz pour les ESC)
PWM_FREQ = 50

FL_ROTOR_PIN = 26
FR_ROTOR_PIN = 19
BL_ROTOR_PIN = 13
BR_ROTOR_PIN = 6

GPIO.setup(FL_ROTOR_PIN, GPIO.OUT)
GPIO.setup(FR_ROTOR_PIN, GPIO.OUT)
GPIO.setup(BL_ROTOR_PIN, GPIO.OUT)
GPIO.setup(BR_ROTOR_PIN, GPIO.OUT)

FL_ROTOR_PWM = GPIO.PWM(FL_ROTOR_PIN, PWM_FREQ)
FR_ROTOR_PWM = GPIO.PWM(FR_ROTOR_PIN, PWM_FREQ)
BL_ROTOR_PWM = GPIO.PWM(BL_ROTOR_PIN, PWM_FREQ)
BR_ROTOR_PWM = GPIO.PWM(BR_ROTOR_PIN, PWM_FREQ)

    # Initialisation (min throttle)
pwm.start(5)
time.sleep(1)
speed = 0 #speed est initialiser à zero puis est update par l'input gui
engine_ctrl(FL_ROTOR_PWM,speed)
engine_ctrl(FR_ROTOR_PWM,speed)
engine_ctrl(BL_ROTOR_PWM,speed)
engine_ctrl(BR_ROTOR_PWM,speed)


#### initialisation mpu6050 ####
#initialisation du capteur et des filtres

MPU = mpu6050(0x68)
kalman_pitch = KalmanFilter()
kalman_roll = KalmanFilter()
DT = 0.01 # delay betwen to mpu mesurment and also use for the PID alog 
time.sleep(1)

#calibration du mpu

accel = MPU.get_accel_data()
pitch = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
roll = math.degrees(math.atan2(accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
kalman_pitch.angle = pitch
kalman_roll.angle = roll



#### boucle principale du rpi ##### 
while True:
    #### controle de vole principale #####
    #récupération des données du mpu
    mpu_data = compas(kalman_pitch, kalman_roll, MPU, DT).split(";") # -> [PITCH, ROLL]
    # calcule decorection pour l'angle pitch
    # variable relatif au besoin de l'algp PID
    pitch_angle_target = 0
    pitch_err_sum = 0
    pitch_err_prev = 0
    pitch_output = PID_ctrl(mpu_data[0], pitch_angle_target,pitch_err_sum,pitch_err_prev ) # -> int output correction pour un moteur

# calcule de corection pour l'angle roll
    # variable relatif au besoin de l'algp PID
    roll_angle_target = 0
    roll_err_sum = 0
    roll_err_prev = 0
    roll_output = PID_ctrl(mpu_data[0], roll_angle_target, roll_err_sum, roll_err_prev ) # -> [speed_FL, speed_FR, speed_BL, speed_BR]
# mixage de a puissance moteur
    FL_motor_speed = speed  - pitch_outup + roll_output
    FR_motor_speed = speed  - pitch_outup - roll_output
    BL_motor_speed = speed  + pitch_outup + roll_output
    BR_motor_speed = speed  + pitch_outup - roll_output


########## arret du drone ######
pwm.stop()
GPIO.cleanup()
print("GPIO nettoyés et PWM arrêté.")
