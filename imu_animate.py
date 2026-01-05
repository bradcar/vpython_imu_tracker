# vpython_imu_tracker.py
"""
Visual tracking of IMU output on Mac using Quaternion transformations.
Quaternions represent rotation with multiple "single axis and rotation angles".

Uses my bno08x library (I2C, SPI, UART) to efficiently read IMU data.
https://github.com/bradcar/bno08x_i2c_spi_MicroPython

Displays Static World axes: Red in X, Green in Y, Blue in Z, RGB in RHS (X, Y, Z).
Vpython display shows X as left & right, Y as top & bottom, and Z as In & out.
Creates Breadboard representation to match IRL to show motion.
Breadboard in white, with Pico mounted on top of board in Green and BNO086 in Red.

The reason we don't use Euler Angles is that have issues with Gimbal Lock.
With Euler angle implementations, some orientations have multiple valid representations.
Quaternions avoid this by providing a unique representation for every possible orientation.
https://en.wikipedia.org/wiki/Gimbal_lock

Credits:
Inspired by Paul McWhorter's instruction videos
9-Axis IMU LESSON 21: Visualizing 3D Rotations in Vpython using Quaternions
https://www.youtube.com/watch?v=S77r-P6YxAU
"""
import math
from time import sleep

import serial
from vpython import *


def create_world_arrows():
    # Static World axes: r in x, green in y, blue in z, RGB in RHS
    arrow(axis=vector(1, 0, 0), length=2, shaftwidth=.05, color=color.red)  # +X
    arrow(axis=vector(0, 1, 0), length=2, shaftwidth=.05, color=color.green)  # +Y
    arrow(axis=vector(0, 0, 1), length=2, shaftwidth=.05, color=color.blue)  # +Z


def create_body_arrows():
    # Dynamic IMU body axes: r in x, green in y, blue in z, RGB in RHS
    arrow_x = arrow(length=3, shaftwidth=.1, color=color.red)
    arrow_y = arrow(length=3, shaftwidth=.1, color=color.green)
    arrow_z = arrow(length=3, shaftwidth=.1, color=color.blue)
    return arrow_x, arrow_y, arrow_z


def create_body_breadboard():
    # Length along X, Height along Y, Width along Z
    breadboard = box(length=6, height=.2, width=2, opacity=0.6, color=color.white)
    # Pico 2 W is toward the left of the board
    pico = box(length=1.7, height=.1, width=.6,
               pos=vector(-2, .15, 0), opacity=0.9, color=color.green)
    # usbc = box(length=.2, height=.1, width=.25,
    #            pos=vector(-2.9, .15, 0), opacity=0.9, color=color.black)
    bno = box(length=1, height=.1, width=.8,
              pos=vector(-.5, .15, 0), opacity=0.9, color=color.red)
    return compound([breadboard, pico, bno])


# Quaternion helper functions - Hamilton names r, i, j, k
def quaternion_rotate(q, v):
    qr, qi, qj, qk = q
    qv = vector(qi, qj, qk)
    return v + 2 * cross(qv, cross(qv, v) + qr * v)


def quaternion_multiply(q1, q2):
    r1, i1, j1, k1 = q1
    r2, i2, j2, k2 = q2
    return (
        r1 * r2 - i1 * i2 - j1 * j2 - k1 * k2,
        r1 * i2 + i1 * r2 + j1 * k2 - k1 * j2,
        r1 * j2 - i1 * k2 + j1 * r2 + k1 * i2,
        r1 * k2 + i1 * j2 - j1 * i2 + k1 * r2
    )


def quaternion_conjugate(q):
    qr, qi, qj, qk = q
    return qr, -qi, -qj, -qk


def quaternion_normalize(qr: float, qi: float, qj: float, qk: float) -> tuple:
    mag_sq = qr ** 2 + qi ** 2 + qj ** 2 + qk ** 2
    if mag_sq < 0.000001:  # Check for near-zero magnitude
        return 1.0, 0.0, 0.0, 0.0
    n = math.sqrt(mag_sq)
    return qr / n, qi / n, qj / n, qk / n


def main():
    scene.range = 5
    scene.forward = vector(-1, -1, -1)  # Camera angle

    create_world_arrows()
    arrow_body_x, arrow_body_y, arrow_body_z = create_body_arrows()
    board_body = create_body_breadboard()

    # streaming data port from Pico/BNO086
    pico = serial.Serial("/dev/cu.usbmodem2101", 115200, timeout=0.05)
    sleep(1)

    # VPython Reference Vectors - the static world axes
    world_x = vector(1, 0, 0)
    world_y = vector(0, 1, 0)
    world_z = vector(0, 0, 1)

    first_q = None

    while True:
        rate(200)
        line = pico.readline()
        if not line: continue

        parts = line.decode(errors="ignore").strip().split(",")
        if len(parts) != 4: continue

        try:
            # Pico 2 W sensor sends: qx, qy, qz, qr
            sensor_qi, sensor_qj, sensor_qk, sensor_qr = map(float, parts)

            # Map Sensor components to VPython's World components,
            # NOTE: REMAP HERE if RE-OREINTATE SENSOR
            # qi (rotation around world X) is Roll
            # qj (rotation around world Y) is Yaw
            # qk (rotation around world Z) is Pitch
            qi = sensor_qj
            qj = sensor_qk
            qk = sensor_qi
            qr = sensor_qr
        except ValueError:
            continue

        q = quaternion_normalize(qr, qi, qj, qk)

        # Set Reference Frame - relative rotation from startup position:
        # Zeroing ensures board starts level on the screen regardless of sensor is oriented at startup
        # q_rel = q_reference_inverse * q_current
        if first_q is None:
            first_q = q
            continue
        q_rel = quaternion_multiply(quaternion_conjugate(first_q), q)

        # Rotate World vectors to find the current Body vectors
        body_x = quaternion_rotate(q_rel, world_x)
        body_y = quaternion_rotate(q_rel, world_y)
        body_z = quaternion_rotate(q_rel, world_z)

        # Update Arrow body, x left/right, y: up/down, Z: in/out
        arrow_body_x.axis = body_x
        arrow_body_y.axis = body_y
        arrow_body_z.axis = body_z

        # Update Breadboard body,  x: left/right, y: up/down
        board_body.axis = body_x
        board_body.up = body_y


if __name__ == "__main__":
    main()
