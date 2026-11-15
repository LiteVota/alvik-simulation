"""Find the green exit of a cell maze using right-hand decisions and real sensors."""

# SPDX-License-Identifier: MPL-2.0

from time import sleep_ms, ticks_diff, ticks_ms

from arduino_alvik import ArduinoAlvik

CELL_CM = 40
OPEN_CM = 22
STOP_CM = 6
SPEED_CM_S = 8
TURN_DEG_S = 60
SETTLE_S = .12
MAX_MOVES = 120
status = {}


class MazeEscape:
    """Scan at cell centres; keep motion feedback separate from path decisions.

    The program knows the cell spacing, not the maze layout or exit coordinates.
    Right/straight/left/back are relative to the direction of the last cell move.
    The forward-facing ToF sensor checks each direction after a measured turn.
    """

    def __init__(self, alvik):
        self.alvik = alvik
        self.heading = self.forward_heading = 0.
        self.previous_time = ticks_ms()
        self.enabled = True
        self.ok_down = False
        self.moves = self.green_frames = 0
        self.previous_message = None
        self._start_scan()

    def _phase(self, phase):
        self.phase = phase
        self.phase_elapsed = 0.

    def _start_scan(self):
        self.choice = 0
        self._aim()

    def _aim(self):
        self.target_heading = self.forward_heading + (-90, 0, 90, 180)[self.choice]
        self._phase('TURN')

    def _finish(self, phase, message):
        self.enabled = False
        self._phase(phase)
        self.alvik.brake()
        self._publish(message)

    def _publish(self, message):
        title = f'Maze Escape | {message} | cells moved: {self.moves}'
        status.update(title=title, hint='Right-hand rule | X pause/resume | C stop | Reload World restarts')
        if title != self.previous_message:
            print('[Maze Escape]', message, '| cells moved:', self.moves)
            self.previous_message = title

    def update(self):
        now = ticks_ms()
        dt = max(0., ticks_diff(now, self.previous_time) / 1000.)
        self.previous_time = now
        # Integrate the preceding frame, including any settling after braking.
        self.heading += self.alvik.get_gyros()[2] * dt
        ok, cancel = self.alvik.get_touch_ok(), self.alvik.get_touch_cancel()
        if self.phase in ('FINISHED', 'FAILED'):
            self.alvik.brake()
            self.ok_down = ok
            return
        if cancel:
            self.enabled = False
        elif ok and not self.ok_down:
            self.enabled = not self.enabled
        self.ok_down = ok
        if not self.enabled:
            self.alvik.brake()
            self.alvik.left_led.set_color(1, 1, 0)
            self.alvik.right_led.set_color(1, 1, 0)
            self._publish('Stopped; X continues')
            return

        self.phase_elapsed += dt
        red, green, blue = self.alvik.get_color()
        on_exit = green > .3 and green > 1.7 * red and green > 1.4 * blue
        self.green_frames = self.green_frames + 1 if on_exit else 0
        if self.green_frames >= 3:
            for led in (self.alvik.left_led, self.alvik.right_led):
                led.set_color(0, 1, 0)
            self._finish('FINISHED', 'EXIT FOUND; reload the world for another run')
            return

        name = ('right', 'straight', 'left', 'back')[self.choice]
        heading_error = self.target_heading - self.heading
        if self.phase == 'TURN':
            if self.phase_elapsed > 8:
                self._fail('Turn timed out')
                return
            if abs(heading_error) <= .8:
                self.alvik.brake()
                self._phase('SETTLE')
            else:
                yaw = max(-TURN_DEG_S, min(TURN_DEG_S, 3 * heading_error))
                self.alvik.drive(0, yaw)
            self._publish('Checking ' + name)
            colour = (0, 0, 1)
        elif self.phase == 'SETTLE':
            self.alvik.brake()
            if self.phase_elapsed >= SETTLE_S:
                front = self.alvik.get_distance()[2]
                if front > OPEN_CM:
                    self.move_start = self.alvik.get_pose()[:2]
                    self._phase('MOVE')
                elif self.choice < 3:
                    self.choice += 1
                    self._aim()
                else:
                    self._fail('No clear passage; check placement and sensor readings')
                    return
            colour = (0, 1, 1)
        else:  # MOVE: wheel odometry measures one cell; gyro feedback holds heading.
            if self.phase_elapsed > 14:
                self._fail('Cell move timed out')
                return
            # The public pose estimate uses wheel odometry, in centimetres.
            pose = self.alvik.get_pose()
            travelled = ((pose[0] - self.move_start[0]) ** 2 +
                         (pose[1] - self.move_start[1]) ** 2) ** .5
            remaining = CELL_CM - travelled
            if remaining <= .1:
                self.alvik.brake()
                self.moves += 1
                self.forward_heading = self.target_heading
                del self.move_start
                if self.moves >= MAX_MOVES:
                    self._fail('Move limit reached; this layout may contain a loop')
                    return
                self._start_scan()
            else:
                front = self.alvik.get_distance()[2]
                if not front > STOP_CM:
                    self._fail('Path blocked during a cell move; reload and check the layout')
                    return
                speed = min(SPEED_CM_S, max(2., .8 * remaining))
                yaw = max(-25, min(25, 5 * heading_error))
                self.alvik.drive(speed, yaw)
                self._publish('Moving ' + name)
            colour = (0, 1, 0)
        self.alvik.left_led.set_color(*colour)
        self.alvik.right_led.set_color(*colour)

    def _fail(self, message):
        for led in (self.alvik.left_led, self.alvik.right_led):
            led.set_color(1, 0, 0)
        self._finish('FAILED', message)


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Maze Escape] Scan right, straight, left, then back; move through the maze and stop on the green exit.')
    print('[Maze Escape] X pause/resume | C stop | X continues | Reload World starts a fresh run')
    print('[Maze Escape] Cell spacing:', CELL_CM, 'cm | no stored map or prewritten route')
    maze = MazeEscape(alvik)
    while True:
        maze.update()
        sleep_ms(20)


if __name__ == '__main__':
    main()
