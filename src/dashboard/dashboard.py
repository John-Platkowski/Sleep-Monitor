"""Live dashboard for the sleep monitor wearable.

Subscribes to the device's BLE characteristic and plots heart rate, motion, and temperature as they
arrive. The firmware notifies once per second, so the plots advance at roughly 1Hz.

Set DEBUG_FAKE_DATA to True to exercise the plotting path with random values and no hardware.

Usage:
    python dashboard.py
"""

import asyncio
import random
import matplotlib.pyplot as plt
from collections import deque
import re
from bleak import BleakClient

# Drives the dashboard with random values instead of connecting over BLE.
DEBUG_FAKE_DATA = False

# Must match the UUIDs in BLEDriver.h.
SERVICE_UUID = "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
CHARACTERISTIC_UUID = "a16cba2a-8165-4039-96c6-06e922eb6551"

# Hardcoded to the development board. Differs per device, and on macOS this is a system-assigned UUID
# rather than a MAC address.
DEVICE_ADDRESS = "EC:E3:34:1C:3B:5E"

# Rolling windows of the most recent samples. Bounded deques discard the oldest automatically, keeping
# the plot to a fixed span and memory flat over a long run.
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
    """Appends one sample to each series and redraws.

    Args:
        hr_val: Filtered heart rate, in BPM.
        motion_val: Peak motion since the last notification, in g with gravity removed.
        temp_val: Mean temperature over the last epoch, in degrees Celsius.
    """
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
    """Parses one BLE notification and plots it.

    The firmware sends ASCII of the form "HR=72.5, Motion=0.03, Temp=36.4". Malformed payloads are
    reported and skipped rather than raising, so a garbled packet does not end the session.

    Args:
        sender: Characteristic that produced the notification. Unused, but required by the bleak
            callback signature.
        data: Raw notification bytes.
    """
    line_in = data.decode('utf-8').strip()
    # Values can be negative (temperature, and the -1 no-reading sentinel), so the sign is part of
    # each group.
    match = re.match(r"HR=(-?[\d.]+), Motion=(-?[\d.]+), Temp=(-?[\d.]+)", line_in)

    if match:
        update_plot(float(match.group(1)), float(match.group(2)), float(match.group(3)))
    else:
        print(f"Unrecognized format: {line_in}")


def run_debug_fake_data():
    """Drives the dashboard with random values, without touching BLE.

    Runs until the plot window closes or the user interrupts. Used to check the plotting path when no
    device is at hand.
    """
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
    """Connects to the device and streams notifications until interrupted.

    Raises:
        BleakError: If the device cannot be found or refuses the connection.
    """
    async with BleakClient(DEVICE_ADDRESS) as client:
        print(f"Connected to {DEVICE_ADDRESS}")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Receiving data...")
        try:
            # Notifications arrive on bleak's callback, so this loop exists only to keep the event
            # loop alive and yield control back to it.
            while True:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping...")


if __name__ == "__main__":
    if DEBUG_FAKE_DATA:
        run_debug_fake_data()
    else:
        asyncio.run(main_ble())