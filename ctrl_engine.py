import RPi.GPIO as GPIO
import time

# Configuration des broches GPIO
ESC_PIN = 26  # Broche GPIO connectée à l'ESC

# Initialisation
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Fréquence PWM (typiquement 50Hz pour les ESC)
PWM_FREQ = 50
pwm = GPIO.PWM(ESC_PIN, PWM_FREQ)

def calibrate_esc():
    """Calibration de l'ESC (à exécuter une seule fois)"""
    print("Début de la calibration...")
    pwm.start(0)
    input("Connectez l'ESC à la batterie et appuyez sur Entrée")
    print("Envoi du signal max...")
    pwm.ChangeDutyCycle(10)  # Max throttle (peut varier selon l'ESC)
    time.sleep(2)
    print("Envoi du signal min...")
    pwm.ChangeDutyCycle(5)   # Min throttle (peut varier selon l'ESC)
    time.sleep(2)
    print("Calibration terminée!")

def set_speed(percent):
    """Définit la vitesse du moteur (0% à 100%)"""
    # Convertir le pourcentage en cycle de duty (typiquement 5-10% pour 50Hz)
    duty_cycle = 5 + (percent / 100.0) * 5
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Vitesse définie à {percent}% (Duty Cycle: {duty_cycle}%)")

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
        set_speed(speed)
        
except KeyboardInterrupt:
    print("Arrêt...")
finally:
    pwm.stop()
    GPIO.cleanup()
    print("GPIO nettoyés et PWM arrêté.")
