import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from collections import deque


PORT = "COM3"
BAUD = 9600
MAX_POINTS = 5000


# Setup serial
ser = serial.Serial(PORT, BAUD, timeout=0.1)


# Setup plot
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget(show=True, title="ADC Plotter")
plot = win.addPlot(title="Real-Time ADC Data")
plot.setYRange(0, 1023)
plot.setLabel('left', 'ADC Value')
plot.setLabel('bottom', 'Sample')
plot.showGrid(x=True, y=True)
curve = plot.plot(pen='y') # Use this to change trace color (I like 'g')


# Data buffer
data = deque(maxlen=MAX_POINTS)


def update():
    # Read serial data:
    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                value = int(line)
                data.append(value)
        except:
            pass
   
    # Update plot
    curve.setData(list(data))


# Timer for updates
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)  # Update every 50ms


pg.exec()
ser.close()
