"""Drive with the directional touch pads. Release to stop."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms

from arduino_alvik import ArduinoAlvik

STOP_CM = 11
SPEED_CM_S = 7
TURN_DEG_S = 58


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Touch Move] Drive with directional controls; forward motion stops when an obstacle is close.')
    enabled = True
    ok_down = blocked = False
    print('[Touch Move] Hold W/A/S/D to drive | X pause/enable | C stop')
    while True:
        ok = alvik.get_touch_ok()
        if alvik.get_touch_cancel():
            enabled = False
        elif ok and not ok_down:
            enabled = not enabled
        ok_down = ok

        speed = yaw = 0
        was_blocked = blocked
        blocked = False
        if enabled:
            speed = SPEED_CM_S * (alvik.get_touch_up() - alvik.get_touch_down())
            yaw = TURN_DEG_S * (alvik.get_touch_left() - alvik.get_touch_right())
            if speed > 0:
                front = min(alvik.get_distance()[1:4])
                blocked = front <= STOP_CM
                if blocked:
                    speed = 0
                    if not was_blocked:
                        print(f'[Touch Move] Forward blocked at {front:.1f} cm')

        alvik.drive(speed, yaw)
        colour = (1, 0, 0) if blocked else (0, 1, 0) if enabled else (1, .45, 0)
        alvik.left_led.set_color(*colour)
        alvik.right_led.set_color(*colour)
        sleep_ms(20)


if __name__ == '__main__':
    main()
