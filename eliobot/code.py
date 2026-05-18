import time

import analogio
import board
import pwmio
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

try:
    from elio import Motors
except ImportError as exc:
    print("Impossible d'importer elio.py ou une de ses dependances:", exc)
    print("Copie elio.py sur CIRCUITPY et ajoute les librairies requises du bundle.")
    raise


# Ajuste ces broches si ton montage differe.
RIGHT_BACKWARD_PIN = board.IO5
RIGHT_FORWARD_PIN = board.IO6
LEFT_BACKWARD_PIN = board.IO7
LEFT_FORWARD_PIN = board.IO8
VBATT_PIN = board.IO4

DEFAULT_SPEED = 40
PWM_FREQUENCY = 1000
BLE_NAME = "ESP32S3-Robot"


right_backward_pwm = pwmio.PWMOut(
    RIGHT_BACKWARD_PIN, frequency=PWM_FREQUENCY, duty_cycle=0
)
right_forward_pwm = pwmio.PWMOut(
    RIGHT_FORWARD_PIN, frequency=PWM_FREQUENCY, duty_cycle=0
)
left_backward_pwm = pwmio.PWMOut(
    LEFT_BACKWARD_PIN, frequency=PWM_FREQUENCY, duty_cycle=0
)
left_forward_pwm = pwmio.PWMOut(
    LEFT_FORWARD_PIN, frequency=PWM_FREQUENCY, duty_cycle=0
)
battery_monitor = analogio.AnalogIn(VBATT_PIN)

motors = Motors(
    right_backward_pwm,
    right_forward_pwm,
    left_backward_pwm,
    left_forward_pwm,
    battery_monitor,
)


def clamp_speed(speed):
    return max(0, min(100, int(speed)))


def parse_speed_arg(parts, default_speed):
    if len(parts) == 1:
        return default_speed

    try:
        return clamp_speed(parts[1])
    except ValueError:
        return None


def apply_raw_drive(left_speed, right_speed):
    if left_speed == 0:
        motors.spin_left_wheel_forward(0)
        motors.spin_left_wheel_backward(0)
    elif left_speed > 0:
        motors.spin_left_wheel_forward(clamp_speed(left_speed))
    else:
        motors.spin_left_wheel_backward(clamp_speed(-left_speed))

    if right_speed == 0:
        motors.spin_right_wheel_forward(0)
        motors.spin_right_wheel_backward(0)
    elif right_speed > 0:
        motors.spin_right_wheel_forward(clamp_speed(right_speed))
    else:
        motors.spin_right_wheel_backward(clamp_speed(-right_speed))


def parse_command(command):
    parts = command.strip().upper().split()
    if not parts:
        return "ERR empty"

    action = parts[0]

    if action == "S":
        motors.slow_stop()
        return "OK stop"

    if action == "F":
        speed = parse_speed_arg(parts, DEFAULT_SPEED)
        if speed is None:
            return "ERR speed"
        motors.move_forward(speed)
        return "OK forward {}".format(speed)

    if action == "B":
        speed = parse_speed_arg(parts, DEFAULT_SPEED)
        if speed is None:
            return "ERR speed"
        motors.move_backward(speed)
        return "OK backward {}".format(speed)

    if action == "L":
        speed = parse_speed_arg(parts, DEFAULT_SPEED)
        if speed is None:
            return "ERR speed"
        motors.turn_left(speed)
        return "OK left {}".format(speed)

    if action == "R":
        speed = parse_speed_arg(parts, DEFAULT_SPEED)
        if speed is None:
            return "ERR speed"
        motors.turn_right(speed)
        return "OK right {}".format(speed)

    if action in ("M", "DRIVE"):
        if len(parts) != 3:
            return "ERR use: M left right"

        try:
            left_speed = max(-100, min(100, int(parts[1])))
            right_speed = max(-100, min(100, int(parts[2])))
        except ValueError:
            return "ERR motor values"

        apply_raw_drive(left_speed, right_speed)
        return "OK motors {} {}".format(left_speed, right_speed)

    if action == "BAT":
        return "OK battery {:.2f}V".format(motors.get_battery_voltage())

    return "ERR unknown"


ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)
advertisement.complete_name = BLE_NAME

rx_buffer = ""

motors.slow_stop()
print("Demarrage du robot BLE avec elio.py...")

while True:
    ble.start_advertising(advertisement)
    print("En attente de connexion Bluetooth...")

    while not ble.connected:
        time.sleep(0.1)

    ble.stop_advertising()
    print("Client connecte")
    uart.write(
        b"Robot connecte.\nCommandes: F [0-100], B [0-100], L [0-100], R [0-100], S, M -100..100 -100..100, BAT\n"
    )

    while ble.connected:
        if uart.in_waiting:
            data = uart.read(uart.in_waiting)
            if data:
                rx_buffer += data.decode("utf-8")

                while "\n" in rx_buffer:
                    line, rx_buffer = rx_buffer.split("\n", 1)
                    response = parse_command(line)
                    print("Commande:", line.strip(), "->", response)
                    uart.write((response + "\n").encode("utf-8"))

        time.sleep(0.01)

    motors.slow_stop()
    rx_buffer = ""
    print("Client deconnecte, arret du robot")
