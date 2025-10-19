import hx711
import time

# Création de l'objet
hx = hx711.HX711(dout_pin=9, pd_sck_pin=11)

# Reset important !
hx.reset()

# Lecture avec moyenne
try:
    for i in range(5):
        values = hx.get_raw_data(times=5)
        average = sum(values) / len(values)
        print(f"Valeurs: {values} | Moyenne: {average}")
        time.sleep(1)
except Exception as e:
    print(f"Erreur: {e}")
