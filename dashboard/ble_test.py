import asyncio
from bleak import BleakClient
from bleak import BleakScanner

DEVICE_NAME = "MyESP32"
SERVICE_UUID = "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
CHARACTERISTIC_UUID = "a16cba2a-8165-4039-96c6-06e922eb6551"

async def main():
    # First scan to find the device address
    
    devices = await BleakScanner.discover()
    esp32_address = None
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            esp32_address = d.address
            print(f"Found ESP32 at {esp32_address}")
            break


    if not esp32_address:
        print("ESP32 not found")
        return

    async with BleakClient(esp32_address) as client:
        print("Connected to ESP32")

        # Read initial value
        data = await client.read_gatt_char(CHARACTERISTIC_UUID)
        print("Initial value:", data.decode("utf-8"))

        # If your ESP32 will notify new data instead of just read:
        def handle_notification(sender, data):
            line_in = data.decode("utf-8").strip()
            print(f"Notification: {line_in}")
            # here you’d parse like your regex and update plots

        await client.start_notify(CHARACTERISTIC_UUID, handle_notification)

        # Keep running
        await asyncio.sleep(60)

asyncio.run(main())
