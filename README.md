# vpython_imu_tracker
using vpython to show IMU orientation

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

## Output Images

Sample Images
<img src="/Users/bradcarlile/Documents/GitHub/vpython_imu_tracker/imgs/vpython-imu-tracker-1.png" width="300"/>


![Output1](/Users/bradcarlile/Documents/GitHub/vpython_imu_tracker/imgs/vpython-imu-tracker-1.png)

![Output2](/Users/bradcarlile/Documents/GitHub/vpython_imu_tracker/imgs/vpython-imu-tracker-2.png)


## Euler Angles, Gimbal Lock, and Quaternions

Euler angles suffer from a well-known issue called Gimbal Lock.
Gimbal lock occurs when two rotation axes align, which removes one degree of freedom. When a degree of freedom is
lost, some orientations will have multiple valid representations.
A well-known example occurred during the Apollo 11 mission.
Quaternions avoid this by providing a unique representation for every possible orientation.
Quaternions represents rotation with multiple "single axis and rotation angles".
Most computer games use this implementation for smooth and predictable graphics.

- https://base.movella.com/s/article/Understanding-Gimbal-Lock-and-how-to-prevent-it?language=en_US
- https://en.wikipedia.org/wiki/Gimbal_lock
- https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
- https://www.youtube.com/watch?v=zjMuIxRvygQ