# vpython_imu_tracker.py
"""
visual tracking of IMU output on mac
Totally inspired by Paul McWhorter instruction videos
9-Axis IMU LESSON 20: Vpython Visualization of Roll, Pitch, and Yaw
https://www.youtube.com/watch?v=7B3KnAj7xzY
"""
import numpy as np
import serial
from time import sleep
from math import sin, cos
from vpython import *

# conversion factors
TO_RAD = 2. * np.pi / 360.
TO_DEG = 1. / TO_RAD
YAW_OFFSET = np.pi


def create_breadboard():
    """create compound breadboard and components"""
    breadboard = box(length=6, width=2, height=.2, opacity=0.5, color=color.white)
    pico = box(length=1.75, width=.6, height=.1, pos=vector(-2, .1 + .05, 0), opacity=0.5, color=color.green)
    bno = box(length=1, width=.75, height=.1, pos=vector(-.5, .1 + .05, 0), opacity=0.5, color=color.red)
    return compound([breadboard, pico, bno])


def create_arrows():
    """create arrows"""
    # Coordinate arrows x red, y green, z blue, RHR
    xlabel = label(text="x", pos=vector(2.2, 0, 0), color=color.red)
    ylabel = label(text="y", pos=vector(0, 2.2, 0), color=color.green)
    zlabel = label(text="z", pos=vector(0, 0, 2.2), color=color.blue)

    # Coordinate arrows: x red, y green, z blue, RHR
    x_arrow = arrow(axis=vector(1, 0, 0), length=2, shaftwidth=.05, color=color.red)
    y_arrow = arrow(axis=vector(0, 1, 0), length=2, shaftwidth=.05, color=color.green)
    z_arrow = arrow(axis=vector(0, 0, 1), length=2, shaftwidth=.05, color=color.blue)

    # IMU arrows x red, y green, z blue, RHR
    right_arrow = arrow(axis=vector(1, 0, 0), length=4, shaftwidth=.1, color=color.red)
    up_arrow = arrow(axis=vector(0, 1, 0), length=4, shaftwidth=.1, color=color.green)
    front_arrow = arrow(axis=vector(0, 0, 1), length=4, shaftwidth=.1, color=color.blue)

    return x_arrow, y_arrow, z_arrow, right_arrow, up_arrow, front_arrow


def main():
    """
    Reads data from bno086 serial port, Must run as main.py on bno086, because USB-C can not share data with Thonny.

    """
    # view screen
    scene.range = 5
    scene.width = 600
    scene.height = 600
    scene.forward = vector(1, 1, 1)

    # create objects and axis arrows
    board = create_breadboard()
    x_arrow, y_arrow, z_arrow, right_arrow, up_arrow, front_arrow = create_arrows()

    # Read data from bno086 serial port, Must run main.py on bno086, because usb-c can not share data with Thonny
    pico_data = serial.Serial("/dev/cu.usbmodem2101", baudrate=115200, timeout=1)
    sleep(5) # wait for tare
    print("Starting data processing...")

    while True:
        rate(50)

        line = pico_data.readline()
        if not line:
            continue

        try:
            roll, pitch, yaw = map(float, line.decode().strip().split(","))
        except ValueError:
            continue

        #print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        roll *= TO_RAD
        pitch *= TO_RAD
        yaw = -yaw * TO_RAD + YAW_OFFSET

        forward = vector(
            cos(yaw) * cos(pitch),
            sin(pitch),
            sin(yaw) * cos(pitch)
        ).norm()

        world_up = vector(0, 1, 0)

        right = cross(forward, world_up).norm()
        up = cross(right, forward).norm()

        front_arrow.axis = forward
        right_arrow.axis = right
        up_arrow.axis = up

        board.axis = forward
        board.up = up

        front_arrow.length = 3
        right_arrow.length = 4
        up_arrow.length = 3


if __name__ == "__main__":
    main()
