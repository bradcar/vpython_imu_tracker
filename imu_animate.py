import numpy as np
import serial
from vpython import *

# conversion factors
TO_RAD = 2. * np.pi / 360.
TO_DEG = 1. / TO_RAD


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


def read_sensor_data(pico_data):
    """read data from serial port"""
    data_packet = pico_data.readline()
    data_packet = str(data_packet, 'utf-8')
    split_packet = data_packet.split(",")
    roll = float(split_packet[0]) * TO_RAD
    pitch = float(split_packet[1]) * TO_RAD
    yaw = float(split_packet[2]) * TO_RAD

    return roll, pitch, yaw


def main():
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
        while pico_data.inWaiting() == 0:  # no data pass
            pass

        roll, pitch, yaw = read_sensor_data(pico_data)

        print(f"roll={roll * TO_DEG:.1f}, pitch={pitch * TO_DEG:.1f}, yaw={yaw * TO_DEG:.1f}")

        k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))

        y = vector(0, 1, 0)
        # cross product RHR (renormalizes to unit vector)
        s = cross(k, y)
        v = cross(s, k)

        front_arrow.axis = k
        right_arrow.axis = s
        up_arrow.axis = v

        board.axis = k
        board.up = v

        front_arrow.length = 3
        right_arrow.length = 4
        up_arrow.length = 3


if __name__ == "__main__":
    main()
