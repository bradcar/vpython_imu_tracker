# vpython_imu_tracker.py  - Runs on Mac
"""
Visual tracking of IMU output on Mac using Quaternion transformations.
Quaternions represent rotation with multiple "single axis and rotation angles".
We are treating the BNO086 quaternion as a body → world rotation.

Suggestion: At startup, the breadboard should be still and flat on a table at script startup. This is because the
first packet defines "Level" for the rest of the session.

Uses my bno08x library (I2C, SPI, UART) to efficiently read IMU data.
https://github.com/bradcar/bno08x_i2c_spi_MicroPython
Pico 2 W outputs driven by quaternion_stream_spi.py

Vpython's static World display axes: Red in X, Green in Y, Blue in Z, RGB in RHS (X, Y, Z).
Vpython display shows X as left & right, Y as top & bottom, and Z as In & out.
Creates Breadboard representation to match IRL to show motion.
Breadboard in white, with Pico mounted on top of board in Green and BNO086 in Red.

The reason we don't use Euler Angles is that have issues with Gimbal Lock.
With Euler angle implementations, some orientations have multiple valid representations.
Quaternions avoid this by providing a unique representation for every possible orientation.
https://en.wikipedia.org/wiki/Gimbal_lock

200 Hz printing of Quaternions over USB-C with 230,400 buadrate (bps) which should have ~ 50% headroom.
Time to read 30 chars

- data comes at 68,000 buadrate/bps = 6,800 * 10bit,
  - assuming (8b data & 2 bits for start & stop)
  - 6,800 = 34 chars /0.005sec (5 ms)
  - up to 34 chars for 4 Quaternions & 2 bytes for "\r\n"
  - Each Quaterion up to 8 chars: pos numbers"0.6643," and negative "-0.0003,"

Earlier code had 100 Hz sensor updates (Quaternions) over USB-C with 115,200 buadrate (bps) which had ~ 50% headroom.

## Vpython Conventions:
Default Axes in Vpython
 - X axis → X+ right screen, X- left screen
 - Y axis → Y+ up screen, Y- down screen
 - Z axis → Z+ toward camera (out) and Z- away from camera (in)

       +Y
       |   -Z
       | /
       o------ +X

Vpython uses Positive rotation Right-Hand Rule about each axis:
 - Around +X: Y moves toward Z.
 - Around +Y: Z moves toward X.
 - Around +Z: X moves toward Y.

A quaternion rotation rotates a vector usin gstandard 3D right-handed rotations.:
    v_rot = q * v * q_conjugate

Where:
 - v is treated as a pure quaternion (0, vx, vy, vz)
 - q must be unit-normalized
 - q_conjugate = (r, -i, -j, -k)

 In practice, VPython-style helper math often uses the Rodrigues' rotation formula optimized vector form:

    v_rot = v + 2 * cross(q_vec, cross(q_vec, v) + q_r * v)

Mappying between VPython and Sensor:
VPython uses a standard Right-Handed System (RHS) for its "World" coordinates.

X-Axis (Red): Points Right and Left.
 * Rotation: Roll. Spinning the object like a screw.

Y-Axis (Green): Points Up and Down.
 * Rotation: Yaw. Turning the object like a compass needle on a table.

Z-Axis (Blue): Points Out of the screen (toward you) and In.
 * Rotation: Pitch. Tilting the nose of the object up or down.

Credits:
Inspired by Paul McWhorter's instruction videos
9-Axis IMU LESSON 21: Visualizing 3D Rotations in Vpython using Quaternions
https://www.youtube.com/watch?v=S77r-P6YxAU
"""
import math
from time import sleep

import serial
from vpython import box, vector, arrow, color, rate, scene, compound, cross


def create_world_arrows():
    """ Static World axes: r in x, green in y, blue in z, RGB in RHS """
    arrow(axis=vector(1, 0, 0), length=2, shaftwidth=.05, color=color.red)  # +X
    arrow(axis=vector(0, 1, 0), length=2, shaftwidth=.05, color=color.green)  # +Y
    arrow(axis=vector(0, 0, 1), length=2, shaftwidth=.05, color=color.blue)  # +Z


def create_body_arrows():
    """ IMU body axes arrows that move with board: r in x, green in y, blue in z, RGB in RHS """
    arrow_x = arrow(length=3, shaftwidth=.1, color=color.red)
    arrow_y = arrow(length=3, shaftwidth=.1, color=color.green)
    arrow_z = arrow(length=3, shaftwidth=.1, color=color.blue)
    return arrow_x, arrow_y, arrow_z


def create_body_breadboard():
    """ Create breadboard image """
    # Length along X, Height along Y, Width along Z
    breadboard = box(length=6, height=.2, width=2, opacity=0.6, color=color.white)

    # Pico 2 W is toward the left of the board
    pico = box(length=1.7, height=.1, width=.6,
               pos=vector(-2, .15, 0), opacity=1.0, color=color.green)

    # usb-c = box(length=.2, height=.1, width=.25,
    #            pos=vector(-2.9, .15, 0), opacity=0.9, color=color.black)

    # show BNO black chip on red sensor board with chip orientation white dot
    bno = box(length=1, height=.1, width=.8,
              pos=vector(-.5, .15, 0), opacity=0.9, color=color.red)
    bno_chip = box(length=.2, height=.1, width=.2,
                   pos=vector(-.45, .20, 0), opacity=1.0, color=color.black)
    bno_orientation = box(length=.05, height=.01, width=.05,
                          pos=vector(-.5, .26, -0.05), opacity=1.0, color=color.white)

    return compound([breadboard, pico, bno, bno_chip, bno_orientation])


# Quaternion helper functions - Hamilton names r, i, j, k
def quaternion_rotate(q, v):
    """
    Rotate vector v by unit quaternion q using optimized Rodrigues formula.
    Equivalent to: q * v * q_conjugate
    """
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
    scene.forward = vector(-1.60972, -0.635176, -0.0731288)

    create_world_arrows()
    arrow_body_x, arrow_body_y, arrow_body_z = create_body_arrows()
    board_body = create_body_breadboard()

    # VPython reference vectors
    world_x, world_y, world_z = vector(1, 0, 0), vector(0, 1, 0), vector(0, 0, 1)

    # Serial connection
    pico = serial.Serial("/dev/cu.usbmodem2101", 230400, timeout=0.05)
    sleep(1)

    first_q = None

    while True:
        rate(400)

        # if vpython lags sensor, flush buffer to catch up
        while pico.in_waiting > 60:
            pico.reset_input_buffer()

        line = pico.readline()
        if not line:
            continue

        parts = line.decode(errors="ignore").strip().split(",")
        if len(parts) != 4:
            continue

        try:
            s_qr, s_qi, s_qj, s_qk = map(float, parts)

            # Re-mapping of BNO sensor to VPython
            # qi(X) = −s_qi  # Roll Negated which changes CW to CCW
            # qj(Y) = s_qk   # Yaw Sensor's "Z" is VPython's "Up"
            # qk(Z) = s_qj   # Pitch Sensor's "Y" is VPython's "Forward"
            qr = s_qr
            qi = -s_qi  # Roll (X)
            qj = s_qk  # Yaw  (Y)
            qk = s_qj  # Pitch (Z)
            q_norm = quaternion_normalize(qr, qi, qj, qk)

            # Assumes sensor is motionless at start and data is correct
            if first_q is None:
                first_q = quaternion_conjugate(q_norm)
                print("\n[INFO] Sensor Zeroed. Starting tracking...")
                continue

            # first_q is conjufate of startup orientation, compute relative rotation in world frame
            q_rel = quaternion_multiply(first_q, q_norm)

            # Update rotating body and arrows attached to it
            body_x = quaternion_rotate(q_rel, world_x)
            body_y = quaternion_rotate(q_rel, world_y)
            body_z = quaternion_rotate(q_rel, world_z)

            arrow_body_x.axis = body_x
            arrow_body_y.axis = body_y
            arrow_body_z.axis = body_z
            board_body.axis = body_x.norm()
            board_body.up = body_y.norm()

        except ValueError:
            continue


if __name__ == "__main__":
    main()
