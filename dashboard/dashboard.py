import serial
import matplotlib.pyplot as plt
from collections import deque

PORT = "/dev/ttyUSB0"   # Linux
# PORT = "COM3"         # Windows
# PORT = "/dev/cu.SLAB_USBtoUART"  # Mac
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)

# keep last 100 values
data = deque(maxlen=100)

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'r-')

while True:
    line_in = ser.readline().decode("utf-8").strip()
    if line_in.startswith("IR:"):
        try:
            value = int(line_in.split(":")[1])
            data.append(value)

            line.set_xdata(range(len(data)))
            line.set_ydata(list(data))
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)
        except ValueError:
            pass
