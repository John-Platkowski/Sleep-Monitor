import serial
import matplotlib.pyplot as plt
from collections import deque
import re

PORT = "/dev/ttyUSB0" # Linux 
# PORT = "COM3" # Windows 
# PORT = "/dev/cu.SLAB_USBtoUART" # Mac
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)

data_ir = deque(maxlen=100)
data_bpm = deque(maxlen=100)
data_avg = deque(maxlen=100)

plt.ion()
fig, ax = plt.subplots()
line_ir, = ax.plot([], [], 'r-', label="IR")
line_bpm, = ax.plot([], [], 'g-', label="BPM")
line_avg, = ax.plot([], [], 'b-', label="Avg BPM")
ax.legend()

while True:
    line_in = ser.readline().decode("utf-8").strip()
    match = re.match(r"IR=(\d+), BPM=([\d.]+), Avg BPM=(\d+)", line_in)
    if match:
        ir_val = int(match.group(1)) // 1000
        bpm_val = float(match.group(2))
        avg_val = int(match.group(3))

        data_ir.append(ir_val)
        data_bpm.append(bpm_val)
        data_avg.append(avg_val)

        line_ir.set_data(range(len(data_ir)), list(data_ir))
        line_bpm.set_data(range(len(data_bpm)), list(data_bpm))
        line_avg.set_data(range(len(data_avg)), list(data_avg))

        ax.relim()
        ax.autoscale_view()

        plt.draw()
        plt.pause(0.01)
