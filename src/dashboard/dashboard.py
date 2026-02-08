import asyncio
import matplotlib.pyplot as plt
from collections import deque
import re
from bleak import BleakClient

# BLE Configuration
SERVICE_UUID = "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
CHARACTERISTIC_UUID = "a16cba2a-8165-4039-96c6-06e922eb6551"
DEVICE_ADDRESS = "EC:E3:34:1C:3B:5E"

# Data storage, deques to limit the sample to 100 values
data_hr = deque(maxlen=100)
data_motion = deque(maxlen=100)
temp_c = None

# Setup plot with two y-axes (heart rate on left, motion on right)
plt.ion()
fig, ax = plt.subplots()
ax2 = ax.twinx()

line_hr, = ax.plot([], [], 'r-', linewidth=2, label="Heart Rate")
line_motion, = ax2.plot([], [], 'm-', alpha=0.7, label="Motion")

ax.set_xlabel("Sample")
ax.set_ylabel("Heart Rate (BPM)", color='r')
ax.tick_params(axis='y', labelcolor='r')
ax2.set_ylabel("Motion Score", color='m')
ax2.tick_params(axis='y', labelcolor='m')

# Combine legends from both axes
lines = [line_hr, line_motion]
labels = [l.get_label() for l in lines]
ax.legend(lines, labels, loc='upper left')

fig.suptitle("Sleep Monitor Dashboard")

# Handle BLE notifications and update plot
def notification_handler(sender, data):
    global temp_c
    line_in = data.decode('utf-8').strip()
    match = re.match(r"HR=([\d\.]+), Motion=([\d\.]+), Temp=([\d\.]+)", line_in)
    
    if match:
        hr_val = float(match.group(1))
        motion_val = float(match.group(2))
        temp_c = float(match.group(3))
        
        data_hr.append(hr_val)
        data_motion.append(motion_val)
        
        line_hr.set_data(range(len(data_hr)), list(data_hr))
        line_motion.set_data(range(len(data_motion)), list(data_motion))
        
        ax.relim()
        ax.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        plt.draw()
        plt.pause(0.01)
    else:
        print(f"Unrecognized format: {line_in}")

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