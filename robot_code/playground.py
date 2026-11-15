"""Drive manually or explore an arena by turning left at obstacles."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms, ticks_diff, ticks_ms

from arduino_alvik import ArduinoAlvik

STOP_CM = 12
SPEED_CM_S = 9
TURN_DEG_S = 58
TURN_ANGLE_DEG = 90
DEFAULT_MODE = 'AUTONOMOUS'
status = {}


class Playground:
    """Keep input responsive while measuring each autonomous turn with the gyro."""

    def __init__(self, alvik):
        self.alvik = alvik
        self.mode = DEFAULT_MODE
        self.enabled = True
        self.turn_remaining = 0.0
        self.ok_down = self.cancel_down = False
        self.previous_time = ticks_ms()
        self.previous_status = None

    def update(self):
        now = ticks_ms()
        elapsed = ticks_diff(now, self.previous_time) / 1000.0
        self.previous_time = now

        # Account for the preceding frame before processing a pause or cancellation.
        if self.enabled and self.turn_remaining > 0:
            self.turn_remaining = max(
                0.0, self.turn_remaining - self.alvik.get_gyros()[2] * elapsed
            )

        ok = self.alvik.get_touch_ok()
        cancel = self.alvik.get_touch_cancel()
        if cancel:
            if not self.cancel_down:
                if not self.enabled:
                    self.mode = 'MANUAL' if self.mode == 'AUTONOMOUS' else 'AUTONOMOUS'
                self.enabled = False
                self.turn_remaining = 0.0
        elif ok and not self.ok_down:
            self.enabled = not self.enabled
        self.ok_down, self.cancel_down = ok, cancel

        speed = yaw = 0.0
        state = 'Stopped'
        colour = (1, 0.45, 0)
        if self.enabled:
            front = min(self.alvik.get_distance()[1:4])
            if self.mode == 'MANUAL':
                speed = SPEED_CM_S * (self.alvik.get_touch_up() - self.alvik.get_touch_down())
                yaw = TURN_DEG_S * (self.alvik.get_touch_left() - self.alvik.get_touch_right())
                state, colour = 'Manual drive', (0, 1, 0)
                # Forward protection leaves reversing and steering available.
                if speed > 0 and front <= STOP_CM:
                    speed = 0.0
                    state, colour = 'Forward blocked', (1, 0, 0)
            else:
                if 0 < self.turn_remaining <= 2:
                    self.turn_remaining = 0.0
                elif self.turn_remaining == 0 and front <= STOP_CM:
                    self.turn_remaining = TURN_ANGLE_DEG
                if self.turn_remaining > 0:
                    # Positive angular velocity is a left turn. Slow near the target.
                    yaw = min(TURN_DEG_S, max(18, 3 * self.turn_remaining))
                    state, colour = 'Turning left', (0, 0.3, 1)
                elif front <= STOP_CM:
                    # Refresh the sensor frame after a turn before moving forward.
                    state, colour = 'Checking clearance', (1, 0, 0)
                else:
                    speed = SPEED_CM_S
                    state, colour = 'Exploring', (0, 0.8, 0.8)

        self.alvik.drive(speed, yaw)
        self.alvik.left_led.set_color(*colour)
        self.alvik.right_led.set_color(*colour)
        message = self.mode + ' | ' + state
        if message != self.previous_status:
            print('[Playground]', message)
            status['title'] = message
            self.previous_status = message


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Robot Playground] Explore the arena with left turns at obstacles, or switch to manual driving and move the props.')
    playground = Playground(alvik)
    print('[Playground] X pause/resume | C stop | C again while stopped changes mode')
    print('[Playground] Manual: hold W/A/S/D; release to stop')
    while True:
        playground.update()
        sleep_ms(20)


if __name__ == '__main__':
    main()
