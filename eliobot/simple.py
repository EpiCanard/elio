import time
import board
import digitalio
import neopixel

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

ble = BLERadio()
BLE_NAME = "Eliobot"
ble.name = BLE_NAME
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)
advertisement.complete_name = BLE_NAME

buzzer_power = digitalio.DigitalInOut(board.IO17)
buzzer_power.direction = digitalio.Direction.OUTPUT
buzzer_power.value = False

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.3)

def blink(color, timer, repeat = 1):
    for i in range (0, repeat):
        pixel.fill(color)
        pixel.show()
        time.sleep(timer)

        pixel.fill((0, 0, 0))
        pixel.show()
        time.sleep(timer)

print("Demarrage BLE")

while True:
    if ble.advertising:
        ble.stop_advertising()

    ble.start_advertising(advertisement)
    print("En attente de connexion BLE")

    while not ble.connected:
        blink((0, 0, 255), 0.1)

    ble.stop_advertising()
    print("Client connecte")
    blink((0, 255, 0), 0.1, 2)

    while ble.connected:
        if uart.in_waiting:
            data = uart.read(uart.in_waiting)
            if data:
                print("Recu:", data)
                try:
                    uart.write(b"Echo: " + data)
                except OSError as exc:
                    print("Envoi BLE impossible:", exc)
        time.sleep(0.05)

    print("Client deconnecte")
    time.sleep(0.5)
