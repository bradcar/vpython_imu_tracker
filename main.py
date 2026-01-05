# test_reports_spi.py - Runs on Pico 2 W with BNO086 sensor outputs over USB-C
#
# BNO08x MicroPython SPI using Uses Brad's bno08x library (I2C, SPI, UART) to efficiently read IMU data.
# https://github.com/bradcar/bno08x_i2c_spi_MicroPython
#
# SPI interface: quaternion output
#
# Enabling reports at 100 Hz (~10 millisec)
# sensor provides frequencies close to what was requested

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms


int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)

bno.quaternion.enable(100)

# sensor provides frequencies at requested 100Hz
#bno.print_report_period()

while True:
    bno.update_sensors()

    if bno.quaternion.updated:
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        print(f"{quat_i:.3f},{quat_j:.3f},{quat_k:.3f},{quat_real:.3f}")

