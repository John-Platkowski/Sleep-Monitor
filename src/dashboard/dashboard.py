import asyncio
import random
import matplotlib.pyplot as plt
from collections import deque
import re
from bleak import BleakClient

# Set True to use random fake data instead of BLE
DEBUG_FAKE_DATA = False

# BLE Configuration
SERVICE_UUID = "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
CHARACTERISTIC_UUID = "a16cba2a-8165-4039-96c6-06e922eb6551"
DEVICE_ADDRESS = "EC:E3:34:1C:3B:5E"

# Data storage, deques to limit the sample to 100 values
data_hr = deque(maxlen=100)
data_motion = deque(maxlen=100)
data_temp = deque(maxlen=100)

# Three separate graphs
plt.ion()
fig, (ax_hr, ax_motion, ax_temp) = plt.subplots(3, 1, sharex=True, figsize=(8, 8))

line_hr, = ax_hr.plot([], [], 'r-', linewidth=2)
ax_hr.set_ylabel("Heart Rate (BPM)")
ax_hr.set_title("Heart Rate")
ax_hr.grid(True, alpha=0.3)

line_motion, = ax_motion.plot([], [], 'm-', linewidth=2)
ax_motion.set_ylabel("Motion Score")
ax_motion.set_title("Motion")
ax_motion.grid(True, alpha=0.3)

line_temp, = ax_temp.plot([], [], 'b-', linewidth=2)
ax_temp.set_ylabel("Temp (C)")
ax_temp.set_xlabel("Sample")
ax_temp.set_title("Temperature")
ax_temp.grid(True, alpha=0.3)

fig.suptitle("Sleep Monitor Dashboard")
fig.tight_layout()

def update_plot(hr_val: float, motion_val: float, temp_val: float):
    # Push one sample into the graphs
    data_hr.append(hr_val)
    data_motion.append(motion_val)
    data_temp.append(temp_val)

    line_hr.set_data(range(len(data_hr)), list(data_hr))
    line_motion.set_data(range(len(data_motion)), list(data_motion))
    line_temp.set_data(range(len(data_temp)), list(data_temp))

    for a in (ax_hr, ax_motion, ax_temp):
        a.relim()
        a.autoscale_view()
    plt.draw()
    plt.pause(0.01)


def notification_handler(sender, data):
    # Handle BLE notifications: parse payload and update plot.
    line_in = data.decode('utf-8').strip()
    match = re.match(r"HR=([\d\.]+), Motion=([\d\.]+), Temp=([\d\.]+)", line_in)

    if match:
        update_plot(float(match.group(1)), float(match.group(2)), float(match.group(3)))
    else:
        print(f"Unrecognized format: {line_in}")


def run_debug_fake_data():
    # Drive the dashboard with random values, disabling BLE
    print("DEBUG: Using fake data (no BLE). Close window or Ctrl+C to stop.")
    try:
        while plt.fignum_exists(fig.number):
            hr = random.uniform(55, 95)
            motion = random.uniform(0, 1.5)
            temp = random.uniform(35.5, 37.5)
            update_plot(hr, motion, temp)
            plt.pause(1.0)  # ~1 s between samples, like BLE notify period
    except KeyboardInterrupt:
        print("\nStopping...")


async def main_ble():
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
    if DEBUG_FAKE_DATA:
        run_debug_fake_data()
    else:
        asyncio.run(main_ble())