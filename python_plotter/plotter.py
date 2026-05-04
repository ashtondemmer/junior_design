import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from collections import deque
from scipy.signal import iirnotch, lfilter, lfilter_zi
import numpy as np


PORT = "COM3"
BAUD = 9600
MAX_POINTS = 5000

# ---- Notch filter settings ----
SAMPLE_RATE = 250.0   # Hz — set this to your Arduino's actual ADC sample rate
NOTCH_FREQ  = 60.0    # Hz — frequency you want to remove (60 in US, 50 in EU)
QUALITY     = 30.0    # Q factor — higher = narrower notch (30 is a good default)

# Design the notch filter once at startup
b, a = iirnotch(w0=NOTCH_FREQ, Q=QUALITY, fs=SAMPLE_RATE)
# Filter state. We seed it from the FIRST real sample on the first update()
# call so the filter starts already in steady state at the signal's DC level
# (otherwise it sees a step from 0 -> ~500 and rings the trace down toward 0).
zi = None


# Setup serial
ser = serial.Serial(PORT, BAUD, timeout=0.1)


# Setup plot
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget(show=True, title="ADC Plotter")
plot = win.addPlot(title=f"Real-Time ADC Data (notch @ {NOTCH_FREQ:.0f} Hz)")
plot.setYRange(0, 1023)
plot.setLabel('left', 'ADC Value')
plot.setLabel('bottom', 'Sample')
plot.showGrid(x=True, y=True)
curve = plot.plot(pen='y')  # Use this to change trace color (I like 'g')


# Data buffer (holds FILTERED samples for plotting)
data = deque(maxlen=MAX_POINTS)


def update():
    global zi

    # Read serial data:
    new_samples = []
    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                new_samples.append(int(line))
        except:
            pass

    # Apply the notch filter to whatever new samples arrived this tick,
    # carrying the filter state `zi` forward so the filter is continuous
    # across update() calls.
    if new_samples:
        if zi is None:
            # Seed filter state to the first sample's DC level so the trace
            # starts at the right value instead of ringing down from 0.
            zi = lfilter_zi(b, a) * float(new_samples[0])
        filtered, zi = lfilter(b, a, np.asarray(new_samples, dtype=float), zi=zi)
        data.extend(filtered)

    # Update plot
    curve.setData(list(data))


# Timer for updates
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)  # Update every 20 ms


pg.exec()
ser.close()