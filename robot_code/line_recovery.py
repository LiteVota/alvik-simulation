"""Follow a line with adjustable proportional gain and recover after line loss."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms, ticks_diff, ticks_ms

from arduino_alvik import ArduinoAlvik

# Compare steering strength at the same base wheel speed (rpm).
GAINS = (10, 30, 50, 80)
BLACK = 150
BASE_RPM = 30
status = {}


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Line Recovery] Compare four steering gains and recover the tape after a short gap.')
    gain = 2
    state = 'WAITING'
    paused = False
    held = set()
    last_error = 0.
    print('[LINE RECOVERY] WAITING | W/S select KP | X start/pause | C cancel | F reset | KP', GAINS[gain])
    while True:
        pads = {name for name in ('ok', 'cancel', 'up', 'down', 'center')
                if getattr(alvik, 'get_touch_' + name)()}
        pressed, held = pads - held, pads
        if 'cancel' in pads:
            state, paused = 'WAITING', False
            alvik.brake()
        elif 'center' in pressed:
            state, paused = 'WAITING', False
            alvik.brake()
            alvik.reset_pose(0, 0, 0)
        elif 'ok' in pressed:
            if state == 'WAITING':
                alvik.reset_pose(0, 0, 0)
                state, last_error = 'FOLLOWING', 0.
                print('[LINE RECOVERY]', state, 'KP', GAINS[gain])
            else:
                paused = not paused
                if paused:
                    alvik.brake()
                elif state == 'SEARCHING':
                    search_started = ticks_ms()
        if state == 'WAITING':
            if 'up' in pressed or 'down' in pressed:
                gain = (gain + (1 if 'up' in pressed else -1)) % len(GAINS)
                print('[LINE RECOVERY] KP', GAINS[gain])
            for led in (alvik.left_led, alvik.right_led):
                led.set_color(0, 0, 1)
        elif not paused:
            left, middle, right = alvik.get_line_sensors()
            if max(left, middle, right) > BLACK:
                if state == 'SEARCHING':
                    state = 'FOLLOWING'
                    print('[LINE RECOVERY] REACQUIRED')
                # Normalize reflectance so the gain scales lateral error.
                last_error = (left - right) / (left + middle + right)
                correction = GAINS[gain] * last_error
                alvik.set_wheels_speed(BASE_RPM - correction, BASE_RPM + correction)
                if correction > .2:
                    colours = ((1, 0, 0), (0, 0, 0))
                elif correction < -.2:
                    colours = ((0, 0, 0), (1, 0, 0))
                else:
                    colours = ((0, 1, 0), (0, 1, 0))
                alvik.left_led.set_color(*colours[0])
                alvik.right_led.set_color(*colours[1])
            else:
                if state != 'SEARCHING':
                    state, search_started = 'SEARCHING', ticks_ms()
                    print('[LINE RECOVERY]', state)
                elapsed = ticks_diff(ticks_ms(), search_started)
                direction = 1 if last_error >= 0 else -1
                # Bridge a short centred gap before searching toward the last sighting.
                if abs(last_error) < .2 and elapsed < 1600:
                    alvik.set_wheels_speed(12, 12)
                elif elapsed < 2300:
                    alvik.set_wheels_speed(-8 * direction, 8 * direction)
                else:
                    alvik.brake()
                    state = 'WAITING'
                    print('[LINE RECOVERY] STOP, line not found')
        display_state = 'PAUSED / ' + state if paused else state
        status.update(state=state, gain=GAINS[gain], paused=paused,
                      title=f'Line Recovery | KP {GAINS[gain]} | {display_state}',
                      hint='W/S gain | X start/pause | C stop | F reset')
        sleep_ms(20)


if __name__ == '__main__':
    main()
