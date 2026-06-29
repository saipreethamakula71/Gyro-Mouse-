import serial
import matplotlib.pyplot as plt
from collections import deque
import numpy as np


# ---------------- SERIAL ----------------
PORT = "/dev/tty.usbserial-0001"
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

# ---------------- MOUSE ----------------

# ---------------- SETTINGS ----------------
MAX_POINTS = 100
WINDOW_SIZE = 10

SENSITIVITY = 0.2
DEADZONE = 2

# ---------------- RAW DATA ----------------
gx_data = deque(maxlen=MAX_POINTS)
gy_data = deque(maxlen=MAX_POINTS)
gz_data = deque(maxlen=MAX_POINTS)

# ---------------- FILTERED DATA ----------------
gx_filtered_data = deque(maxlen=MAX_POINTS)
gy_filtered_data = deque(maxlen=MAX_POINTS)
gz_filtered_data = deque(maxlen=MAX_POINTS)

# ---------------- HISTORY ----------------
hangle = 0
vangle = 0

hangle_history = deque(maxlen=MAX_POINTS)
vangle_history = deque(maxlen=MAX_POINTS)

# ---------------- FILTER ----------------
def moving_average(input_data, output_data):

    if len(input_data) == 0:
        return

    if len(input_data) >= WINDOW_SIZE:
        avg = np.mean(list(input_data)[-WINDOW_SIZE:])
    else:
        avg = np.mean(list(input_data))

    output_data.append(avg)

# ---------------- INTERACTIVE PLOTS ----------------
plt.ion()

# Raw Gyroscope
fig1, (gyro1, gyro2, gyro3) = plt.subplots(3, 1, figsize=(8, 8))
fig1.suptitle("Raw Gyroscope")

# Filtered Gyroscope
fig2, (proc1, proc2, proc3) = plt.subplots(3, 1, figsize=(8, 8))
fig2.suptitle("Filtered Gyroscope")

# Mouse Outputs
fig3, (fin1, fin2) = plt.subplots(2, 1, figsize=(8, 6))
fig3.suptitle("Mouse Output")

# =====================================================
# MAIN LOOP
# =====================================================

while True:

    line = ser.readline().decode(errors="ignore").strip()

    if not line.startswith("DATA"):
        continue

    try:

        vals = line.split(",")

        gx = float(vals[4])
        gy = float(vals[5])
        gz = float(vals[6])

        # ---------------- STORE RAW ----------------

        gx_data.append(gx)
        gy_data.append(gy)
        gz_data.append(gz)

        # ---------------- FILTER ----------------

        moving_average(gx_data, gx_filtered_data)
        moving_average(gy_data, gy_filtered_data)
        moving_average(gz_data, gz_filtered_data)

        if len(gx_filtered_data) == 0:
            continue

        # ---------------- MOUSE CONTROL ----------------

        horizontal_velocity = gz_filtered_data[-1]
        vertical_velocity = gx_filtered_data[-1]

        if abs(horizontal_velocity) < DEADZONE:
            horizontal_velocity = 0

        if abs(vertical_velocity) < DEADZONE:
            vertical_velocity = 0

        dx = horizontal_velocity * SENSITIVITY
        dy = vertical_velocity * SENSITIVITY

        #pyautogui.moveRel(dx, dy, duration=0)
        print(dx,dy)

        # ---------------- HISTORY ----------------

        hangle += horizontal_velocity * 0.1
        vangle += vertical_velocity * 0.1

        hangle = max(min(hangle, 180), -180)
        vangle = max(min(vangle, 180), -180)

        hangle_history.append(hangle)
        vangle_history.append(vangle)

        # ---------------- CLEAR PLOTS ----------------

        gyro1.clear()
        gyro2.clear()
        gyro3.clear()

        proc1.clear()
        proc2.clear()
        proc3.clear()

        fin1.clear()
        fin2.clear()

        # ---------------- RAW GYRO ----------------

        gyro1.plot(gx_data)
        gyro1.set_title("GX")
        gyro1.set_ylim(-250, 250)

        gyro2.plot(gy_data)
        gyro2.set_title("GY")
        gyro2.set_ylim(-250, 250)

        gyro3.plot(gz_data)
        gyro3.set_title("GZ")
        gyro3.set_ylim(-250, 250)

        # ---------------- FILTERED ----------------

        proc1.plot(gx_filtered_data)
        proc1.set_title("Filtered GX")
        proc1.set_ylim(-250, 250)

        proc2.plot(gy_filtered_data)
        proc2.set_title("Filtered GY")
        proc2.set_ylim(-250, 250)

        proc3.plot(gz_filtered_data)
        proc3.set_title("Filtered GZ")
        proc3.set_ylim(-250, 250)

        # ---------------- MOUSE OUTPUT ----------------

        fin1.plot(hangle_history)
        fin1.set_title("Horizontal")

        fin2.plot(vangle_history)
        fin2.set_title("Vertical")

        fig1.tight_layout()
        fig2.tight_layout()
        fig3.tight_layout()

        plt.pause(0.01)

    except Exception as e:
        print("Error:", e)