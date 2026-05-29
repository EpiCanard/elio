import argparse
import asyncio
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from bleak.exc import BleakError

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> robot
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # robot -> PC

ROBOT_NAME = "Eliobot"


def has_uart_service(advertisement_data: AdvertisementData) -> bool:
    return UART_SERVICE_UUID in {
        service_uuid.lower() for service_uuid in advertisement_data.service_uuids
    }


def get_display_name(device: BLEDevice, advertisement_data: AdvertisementData) -> str:
    return advertisement_data.local_name or device.name or "<sans nom>"


async def find_robot(name: str, timeout: float) -> Optional[BLEDevice]:
    print(f"Scan BLE pendant {timeout:.0f}s...")

    def match_robot(device: BLEDevice, advertisement_data: AdvertisementData) -> bool:
        display_name = get_display_name(device, advertisement_data)
        if display_name == name or has_uart_service(advertisement_data):
            services = ", ".join(advertisement_data.service_uuids) or "aucun UUID annonce"
            print(f"- candidat: {display_name} [{device.address}] services: {services}")
            return True

        print(f"- ignore: {display_name} [{device.address}]")
        return False

    return await BleakScanner.find_device_by_filter(match_robot, timeout=timeout)


def on_disconnect(_: BleakClient) -> None:
    print("Deconnexion BLE signalee par BlueZ/le robot")


async def wait_until_done(client: BleakClient, seconds: float) -> None:
    if seconds > 0:
        await asyncio.sleep(seconds)
        return

    print("Connexion maintenue. Appuie sur Ctrl+C pour quitter.")
    while client.is_connected:
        await asyncio.sleep(1)


async def run_client(args: argparse.Namespace) -> None:
    target = await find_robot(args.name, args.scan_timeout)
    if target is None:
        print(f"Robot '{args.name}' introuvable")
        return

    print(f"Connexion a {target.address}...")
    try:
        async with BleakClient(
            target,
            disconnected_callback=on_disconnect,
            services=[UART_SERVICE_UUID],
            timeout=args.connect_timeout,
        ) as client:
            print("Connecte")

            services = [service.uuid for service in client.services]
            print("Services GATT:", ", ".join(services) or "aucun")

            def on_rx(_: int, data: bytearray) -> None:
                try:
                    print("Robot:", data.decode("utf-8"), end="")
                except UnicodeDecodeError:
                    print("Robot bytes:", bytes(data))

            await client.start_notify(UART_TX_UUID, on_rx)

            message = args.message.rstrip("\n") + "\n"
            print("Envoi:", message.strip())
            await client.write_gatt_char(UART_RX_UUID, message.encode("utf-8"))

            await wait_until_done(client, args.wait)
            if client.is_connected:
                await client.stop_notify(UART_TX_UUID)
            print("Deconnecte")
    except BleakError as exc:
        print(f"Erreur Bleak: {exc}")
        print(
            "Sous Arch/BlueZ, si l'erreur est 'failed to discover services', "
            "essaie: bluetoothctl remove "
            f"{target.address}, puis redemarre l'ESP32-S3 et relance ce script."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Client BLE Nordic UART pour Eliobot")
    parser.add_argument("--name", default=ROBOT_NAME)
    parser.add_argument("--message", default="0 0")
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    parser.add_argument(
        "--wait",
        type=float,
        default=0.0,
        help="Secondes avant de deconnecter. 0 = rester connecte jusqu'a Ctrl+C.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    asyncio.run(run_client(parse_args()))
