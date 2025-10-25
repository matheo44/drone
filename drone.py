import RPi.GPIO as GPIO
import time



def engine_ctrl(pwm, trust):

    """Définit la vitesse du moteur (0% à 100%)"""
    # Convertir le pourcentage en cycle de duty (typiquement 5-10% pour 50Hz)
    duty_cycle = 5 + ( trust / 100.0) * 5
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Vitesse définie à {trust}% (Duty Cycle: {duty_cycle}%)")


# Fréquence PWM (typiquement 50Hz pour les ESC)
ESC_PIN = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)
PWM_FREQ = 50
pwm = GPIO.PWM(ESC_PIN, PWM_FREQ)

try:
    # Calibration (à faire une seule fois)
#    calibrate_esc()
    # Initialisation (min throttle)
    pwm.start(5)
    time.sleep(1)
    
   # Exemple de contrôle
    while True:
        speed = int(input("Entrez la vitesse (0-100, 'q' pour quitter): "))
        if speed == 'q':
            break
        engine_ctrl(pwm,speed)
        
except KeyboardInterrupt:
    print("Arrêt...")
finally:
    pwm.stop()
    GPIO.cleanup()
    print("GPIO nettoyés et PWM arrêté.")
