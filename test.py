
import time
import PyQMC


sensor = PyQMC.QMC5883L()


while True:
    m = sensor.get_magnet_raw()
    b = sensor.get_bearing_raw()
    print(f"\n>> bearing: {b:.4f}")
    print(f">> magnet:  {m}")
    time.sleep(1)