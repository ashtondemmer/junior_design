"""
Measures the actual sample rate of the PIC24 serial stream.

Run it, let it sit for ~10-15 seconds, then read the average rate it prints.
Plug that number into SAMPLE_RATE in ekg_plotter_notch.py.
"""

import serial
import time

PORT = "COM3"
BAUD = 9600
DURATION_S = 10.0  # how long to measure for

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# Throw away whatever is already buffered + the first partial line.
ser.reset_input_buffer()
ser.readline()

print(f"Measuring for {DURATION_S:.0f} seconds...")

count = 0
last_print = time.time()
t_start  = time.time()

while time.time() - t_start < DURATION_S:
    line = ser.readline().decode(errors='ignore').strip()
    if not line:
        continue
    try:
        int(line)         # only count lines that are valid samples
        count += 1
    except ValueError:
        continue

    # Print a running estimate once per second so you can see it stabilize.
    now = time.time()
    if now - last_print >= 1.0:
        rate = count / (now - t_start)
        print(f"  {now - t_start:5.1f}s elapsed | {count} samples | ~{rate:.1f} Hz")
        last_print = now

elapsed = time.time() - t_start
avg_rate = count / elapsed

print()
print(f"Total samples: {count} in {elapsed:.2f} s")
print(f"Average sample rate: {avg_rate:.1f} Hz")
print()
print(f"-> Set SAMPLE_RATE = {avg_rate:.1f} in your plotter.")

ser.close()