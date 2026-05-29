import time

import analogio
import board
import neopixel
import pwmio
import digitalio

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from elio import Motors

BLE_NAME = "Eliobot"
PWM_FREQUENCY = 1000
MAX_ANGLE = 90
SEND_DRIVE_ACKS = False
DEBUG_DRIVE_COMMANDS = False

buzzer_power = digitalio.DigitalInOut(board.IO17)
buzzer_power.direction = digitalio.Direction.OUTPUT
buzzer_power.value = False

AIN1 = pwmio.PWMOut(board.IO36, frequency=PWM_FREQUENCY, duty_cycle=0)
AIN2 = pwmio.PWMOut(board.IO38, frequency=PWM_FREQUENCY, duty_cycle=0)
BIN1 = pwmio.PWMOut(board.IO35, frequency=PWM_FREQUENCY, duty_cycle=0)
BIN2 = pwmio.PWMOut(board.IO37, frequency=PWM_FREQUENCY, duty_cycle=0)
vBatt_pin = analogio.AnalogIn(board.BATTERY)

motors = Motors(AIN1, AIN2, BIN1, BIN2, vBatt_pin)

ble = BLERadio()
ble.name = BLE_NAME
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)
advertisement.complete_name = BLE_NAME

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.3)
rx_buffer = ""


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def blink(color, delay, repeat=1):
    for _ in range(repeat):
        pixel.fill(color)
        pixel.show()
        time.sleep(delay)

        pixel.fill((0, 0, 0))
        pixel.show()
        time.sleep(delay)


def parse_drive_command(command):
    normalized = command.strip().replace(",", " ")
    parts = normalized.split()

    if len(parts) != 2:
        return None

    try:
        angle = float(parts[0])
        speed = float(parts[1])
    except ValueError:
        return None

    angle = clamp(angle, -MAX_ANGLE, MAX_ANGLE)
    speed = clamp(speed, -100, 100)
    return angle, speed


def set_wheel_speed(left_speed, right_speed):
    left_speed = clamp(left_speed, -100, 100)
    right_speed = clamp(right_speed, -100, 100)

    if left_speed > 0:
        motors.spin_left_wheel_forward(left_speed)
    elif left_speed < 0:
        motors.spin_left_wheel_backward(-left_speed)
    else:
        motors.BIN1.duty_cycle = 0
        motors.BIN2.duty_cycle = 0

    if right_speed > 0:
        motors.spin_right_wheel_forward(right_speed)
    elif right_speed < 0:
        motors.spin_right_wheel_backward(-right_speed)
    else:
        motors.AIN1.duty_cycle = 0
        motors.AIN2.duty_cycle = 0


def drive(angle, speed):
    if speed == 0:
        motors.slow_stop()
        return 0, 0

    steering = angle / MAX_ANGLE

    if steering < 0:
        left_speed = speed * (1 + steering)
        right_speed = speed
    else:
        left_speed = speed
        right_speed = speed * (1 - steering)

    set_wheel_speed(left_speed, right_speed)
    return left_speed, right_speed


def send_line(message):
    try:
        uart.write((message + "\n").encode("utf-8"))
    except OSError as exc:
        print("Envoi BLE impossible:", exc)


motors.slow_stop()
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
    send_line("OK commandes: angle vitesse, ex: -30 50")

    while ble.connected:
        if uart.in_waiting:
            data = uart.read(uart.in_waiting)
            if data:
                try:
                    rx_buffer += data.decode("utf-8")
                except UnicodeError:
                    send_line("ERR utf8")
                    continue

                while "\n" in rx_buffer:
                    line, rx_buffer = rx_buffer.split("\n", 1)
                    command = parse_drive_command(line)

                    if command is None:
                        send_line("ERR format: angle vitesse")
                        continue

                    angle, speed = command
                    left_speed, right_speed = drive(angle, speed)
                    if DEBUG_DRIVE_COMMANDS:
                        print(
                            "Commande:",
                            line.strip(),
                            "-> angle",
                            angle,
                            "vitesse",
                            speed,
                            "gauche",
                            left_speed,
                            "droite",
                            right_speed,
                        )
                    if SEND_DRIVE_ACKS:
                        send_line(
                            "OK angle {:.1f} vitesse {:.1f} gauche {:.1f} droite {:.1f}".format(
                                angle, speed, left_speed, right_speed
                            )
                        )

        time.sleep(0.02)

    motors.slow_stop()
    print("Client deconnecte")
    time.sleep(0.5)
