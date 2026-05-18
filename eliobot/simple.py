import time
import board

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

ble = BLERadio()
ble.name = "Eliobot"
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)

led = getattr(board, "LED", None)

def set_led(state):
    if led is None:
        return
    import digitalio
    global led_io
    try:
        led_io
    except NameError:
        led_io = digitalio.DigitalInOut(led)
        led_io.direction = digitalio.Direction.OUTPUT
    led_io.value = state

print("Demarrage BLE")
ble.start_advertising(advertisement)

while True:
    while not ble.connected:
        set_led(False)
        time.sleep(0.1)

    print("Client connecte")
    set_led(True)
    uart.write("Bonjour depuis Eliobot BLE!\r\n")

    while ble.connected:
        if uart.in_waiting:
            data = uart.read(uart.in_waiting)
            if data:
                print("Recu:", data)
                uart.write(b"Echo: " + data)
        time.sleep(0.05)

    print("Client deconnecte")
    set_led(False)
    ble.start_advertising(advertisement)
