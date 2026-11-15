
"""Move towards or away from a centred target to hold a 10 cm gap."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms

from arduino_alvik import ArduinoAlvik

TARGET_CM = 10
MAX_TRACK_CM = 55


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Hand Follower] Move forward or backward to keep a 10 cm gap to the target in front.')
    for led in (alvik.left_led, alvik.right_led):
        led.set_color(0, 1, 0)
    running = True
    ok_down = False
    print('Hand follower | X pause/resume | C cancel | J nearer | K farther')
    while True:
        ok = alvik.get_touch_ok()
        if alvik.get_touch_cancel():
            running = False
        elif ok and not ok_down:
            running = not running
        ok_down = ok
        cm = alvik.get_distance()[2]
        if running and 0 < cm <= MAX_TRACK_CM and abs(cm - TARGET_CM) > .6:
            alvik.drive(max(-8, min(8, 1.5 * (cm - TARGET_CM))), 0)
        else:
            alvik.brake()
        sleep_ms(20)


if __name__ == '__main__':
    main()
