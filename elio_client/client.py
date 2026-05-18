import asyncio
from bleak import BleakScanner, BleakClient

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # PC -> robot
UART_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # robot -> PC

ROBOT_NAME = "Eliobot"

async def main():
    print("Scan BLE...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = None
    for d in devices:
        print(f"- {d.name} [{d.address}]")
        if d.name == ROBOT_NAME:
            target = d

    if target is None:
        print(f"Robot '{ROBOT_NAME}' introuvable")
        return

    print(f"Connexion a {target.name}...")
    async with BleakClient(target) as client:
        print("Connecte")

        def on_rx(_, data: bytearray):
            try:
                print("Robot:", data.decode("utf-8"), end="")
            except UnicodeDecodeError:
                print("Robot bytes:", bytes(data))

        await client.start_notify(UART_TX_UUID, on_rx)

        message = "bonjour robot\n"
        print("Envoi:", message.strip())
        await client.write_gatt_char(UART_RX_UUID, message.encode("utf-8"))

        await asyncio.sleep(10)

        await client.stop_notify(UART_TX_UUID)
        print("Deconnecte")

asyncio.run(main())
