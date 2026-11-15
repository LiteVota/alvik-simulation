
"""Black tape demo. The left/centre/right readings steer the wheels."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms

from arduino_alvik import ArduinoAlvik


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Black Line] Follow black tape with three line sensors and stop when the track ends.')
    for led in (alvik.left_led, alvik.right_led):
        led.set_color(0, 0, 1)
    running = True
    ok_down = False
    lost = 0
    error = 0
    print('Black line | X pause/resume | C cancel')
    while True:
        ok = alvik.get_touch_ok()
        if alvik.get_touch_cancel():
            running = False
        elif ok and not ok_down:
            running = not running
        ok_down = ok
        if not running:
            alvik.brake()
        if running:
            left, centre, right = alvik.get_line_sensors()
            if max(left, centre, right) > 150:
                error = (left - right) / (left + centre + right)
                alvik.set_wheels_speed(28 - 43 * error, 28 + 43 * error)
                lost = 0
            elif lost < 20:
                lost += 1
                alvik.set_wheels_speed(12 - 25 * error, 12 + 25 * error)
            else:
                running = False
                alvik.brake()
                print('[LINE] End of tape or line lost')
        sleep_ms(20)


if __name__ == '__main__':
    main()
