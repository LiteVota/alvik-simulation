# SPDX-License-Identifier: MPL-2.0
from arduino_alvik import ArduinoAlvik
from time import sleep_ms

alvik = ArduinoAlvik()
alvik.begin()

while True:
    # Write your Alvik code here.
    sleep_ms(20)
