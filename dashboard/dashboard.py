import asyncio
import matplotlib.pyplot as plt
from collections import deque
import re
from bleak import BleakClient

# BLE Configuration
SERVICE_UUID = "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
CHARACTERISTIC_UUID = "a16cba2a-8165-4039-96c6-06e922eb6551"
DEVICE_ADDRESS = "EC:E3:34:1C:3B:5E"

# Data storage
data_ir = deque(maxlen=100)
data_bpm = deque(maxlen=100)
data_avg = deque(maxlen=100)

# Setup plot
plt.ion()
fig, ax = plt.subplots()
line_ir, = ax.plot([], [], 'r-', label="IR")
line_bpm, = ax.plot([], [], 'g-', label="BPM")
line_avg, = ax.plot([], [], 'b-', label="Avg BPM")
ax.legend()

# Handle BLE notifications and update plot
def notification_handler(sender, data):
    line_in = data.decode('utf-8').strip()
    match = re.match(r"IR=(\d+), BPM=([\d\.]+), Avg BPM=(\d+)", line_in)
    
    if match:
        ir_val = int(match.group(1)) // 1000
        bmp_val = float(match.group(2))
        avg_val = int(match.group(3))
        
        data_ir.append(ir_val)
        data_bpm.append(bmp_val)
        data_avg.append(avg_val)
        
        line_ir.set_data(range(len(data_ir)), list(data_ir))
        line_bpm.set_data(range(len(data_bpm)), list(data_bpm))
        line_avg.set_data(range(len(data_avg)), list(data_avg))
        
        ax.relim()
        ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

async def main():
    async with BleakClient(DEVICE_ADDRESS) as client:
        print(f"Connected to {DEVICE_ADDRESS}")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Receiving data...")
        
        try:
            while True:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping...")

if __name__ == "__main__":
    asyncio.run(main())