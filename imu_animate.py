# vpython_imu_tracker_fixed_remap.py
"""
Visual tracking of IMU output on Mac
Totally inspired by Paul McWhorter instruction videos
9-Axis IMU LESSON 20: VPython Visualization of Roll, Pitch, and Yaw
https://www.youtube.com/watch?v=7B3KnAj7xzY
"""
import math
from time import sleep

import serial
from vpython import *


def create_world_arrows():
    # World axes (Static)
    arrow(axis=vector(1, 0, 0), length=2, shaftwidth=.05, color=color.red)  # +X
    arrow(axis=vector(0, 1, 0), length=2, shaftwidth=.05, color=color.green)  # +Y
    arrow(axis=vector(0, 0, 1), length=2, shaftwidth=.05, color=color.blue)  # +Z


def create_body_arrows():
    # IMU body axes (Dynamic)
    x_body = arrow(length=3, shaftwidth=.1, color=color.red)
    y_body = arrow(length=3, shaftwidth=.1, color=color.green)
    z_body = arrow(length=3, shaftwidth=.1, color=color.blue)
    return x_body, y_body, z_body


def create_body_breadboard():
    # Length along X, Height along Y, Width along Z
    breadboard = box(length=6, height=.2, width=2, opacity=0.5, color=color.white)
    pico = box(length=1.75, height=.1, width=.6,
               pos=vector(-2, .15, 0), opacity=0.5, color=color.green)
    bno = box(length=1, height=.1, width=.75,
              pos=vector(-.5, .15, 0), opacity=0.5, color=color.red)
    return compound([breadboard, pico, bno])


# Quat helpers
def quat_rotate(q, v):
    w, x, y, z = q
    qv = vector(x, y, z)
    return v + 2 * cross(qv, cross(qv, v) + w * v)


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    )


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def main():
    scene.range = 5
    scene.forward = vector(-1, -1, -1)  # Camera angle

    create_world_arrows()
    x_body, y_body, z_body = create_body_arrows()
    board = create_body_breadboard()

    # streaming data port set
    pico = serial.Serial("/dev/cu.usbmodem2101", 115200, timeout=0.05)
    sleep(1)

    # VPython Reference Vectors
    # We want X to be Forward, Y to be Up, Z to be Right/Left
    forward0 = vector(1, 0, 0)
    up0 = vector(0, 1, 0)
    side0 = vector(0, 0, 1)

    first_q = None

    while True:
        rate(100)
        line = pico.readline()
        if not line: continue

        parts = line.decode(errors="ignore").strip().split(",")
        if len(parts) != 4: continue

        try:
            raw_x, raw_y, raw_z, raw_w = map(float, parts)

            # Swapping components to align Sensor -> VPython
            qx = raw_y
            qy = raw_z
            qz = -raw_x
            qw = raw_w
        except ValueError:
            continue

        # normalize Quat
        n = math.sqrt(qw ** 2 + qx ** 2 + qy ** 2 + qz ** 2)
        q = (qw / n, qx / n, qy / n, qz / n)

        # Set Reference Frame
        if first_q is None:
            first_q = q
            continue

        # Remove the "tilt" of your desk if it wasn't level
        q_rel = quat_mul(quat_conjugate(first_q), q)

        # Rotate Basis Vectors - where IMU is pointing in 3d space
        v_forward = quat_rotate(q_rel, forward0)
        v_up = quat_rotate(q_rel, up0)
        v_side = quat_rotate(q_rel, side0)

        # x_body shows the Forward direction
        x_body.axis = v_forward
        y_body.axis = v_up
        z_body.axis = v_side

        # board.axis is the length of the board (Forward)
        # board.up is the top face of the board (Up)
        board.axis = v_forward
        board.up = v_up


if __name__ == "__main__":
    main()
