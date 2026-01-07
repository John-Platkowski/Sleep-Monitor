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
data_ir = deque(maxlen=100)
data_bpm = deque(maxlen=100)
data_avg = deque(maxlen=100)
data_motion = deque(maxlen=100)

# Setup plot with two y-axes (heart rate data on left, motion on right)
plt.ion()
fig, ax = plt.subplots()
ax2 = ax.twinx()

line_ir, = ax.plot([], [], 'r-', label="IR")
line_bpm, = ax.plot([], [], 'g-', label="BPM")
line_avg, = ax.plot([], [], 'b-', label="Avg BPM")
line_motion, = ax2.plot([], [], 'm-', label="Motion")

ax.set_ylabel("IR / BPM")
ax2.set_ylabel("Motion Score", color='m')
ax2.tick_params(axis='y', labelcolor='m')

# Combine legends from both axes
lines = [line_ir, line_bpm, line_avg, line_motion]
labels = [l.get_label() for l in lines]
ax.legend(lines, labels, loc='upper left')

# Handle BLE notifications and update plot
def notification_handler(sender, data):
    line_in = data.decode('utf-8').strip()
    match = re.match(r"IR=(\d+), BPM=([\d\.]+), Avg BPM=(\d+), Motion=([\d\.]+)", line_in)
    
    if match:
        ir_val = int(match.group(1)) // 1000
        bpm_val = float(match.group(2))
        avg_val = int(match.group(3))
        motion_val = float(match.group(4))
        
        data_ir.append(ir_val)
        data_bpm.append(bpm_val)
        data_avg.append(avg_val)
        data_motion.append(motion_val)
        
        line_ir.set_data(range(len(data_ir)), list(data_ir))
        line_bpm.set_data(range(len(data_bpm)), list(data_bpm))
        line_avg.set_data(range(len(data_avg)), list(data_avg))
        line_motion.set_data(range(len(data_motion)), list(data_motion))
        
        ax.relim()
        ax.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
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