"""Button/LED metro control using rendered sensors, gyro and wheel encoders.

Cancel cycles while docked. OK starts or pauses/resumes motion.
The supplied star network has one unique path between any two terminals.
"""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms, ticks_diff, ticks_ms
import math

from arduino_alvik import ArduinoAlvik

COLOURS = ('CYAN', 'MAGENTA', 'YELLOW')
RGB = {'CYAN': (0., 1., 1.), 'MAGENTA': (1., 0., 1.), 'YELLOW': (1., 1., 0.)}
BEARINGS = {'CYAN': math.pi, 'MAGENTA': math.pi / 3, 'YELLOW': -math.pi / 3}


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def label(rgb):
    r, g, b = rgb
    if max(rgb) < .25:
        return 'BLACK'
    if max(rgb) - min(rgb) < .16:
        return 'WHITE'
    if r < .65 * min(g, b):
        return 'CYAN'
    if g < .65 * min(r, b):
        return 'MAGENTA'
    if b < .65 * min(r, g):
        return 'YELLOW'
    return 'OTHER'


class Metro:
    def __init__(self, alvik, start='CYAN'):
        self.alvik = alvik
        self.current = self.route = self.selected = self.destination = start
        self.state = 'DOCKED'
        self.inbound = True  # Supplied world starts facing Central.
        self.arrivals, self.transfers, self.transitions = [], [], []
        self.route_plan = []
        self.error = ''
        self.time = self.deadline = self.distance = self.phase_distance = 0.
        self.last_wheels = self.alvik.get_wheels_position('rad')
        self.yaw = wrap(BEARINGS[start] + math.pi)
        self.last_error = 0.
        self.lost_since = self.marker_since = None
        self.armed = False
        self.resume_state = None
        self._ok_down = self._cancel_down = False
        self.note('Ready: C cycles while docked | X starts or pauses/resumes')
        self.update_leds()

    def note(self, text):
        print('[METRO] ' + text)

    def enter(self, state, timeout=40.):
        self.state = state
        self.deadline = self.time + timeout
        self.phase_distance = self.distance
        self.lost_since = self.marker_since = None
        self.transitions.append((round(self.time, 2), state, self.route))
        self.note(state + ' | line ' + self.route + ' | goal ' + self.destination)

    def update_leds(self):
        colour = RGB[self.selected if self.state == 'DOCKED' else self.destination]
        if self.state == 'FAULT':
            colour = (1., 0., 0.) if int(self.time / .15) % 2 == 0 else (0., 0., 0.)
        elif self.state in ('DOCKED', 'PAUSED'):
            interval = .30 if self.state == 'DOCKED' else .60
            if int(self.time / interval) % 2:
                colour = (0., 0., 0.)
        for led in (self.alvik.left_led, self.alvik.right_led):
            led.set_color(*colour)

    def sample_odometry(self, dt):
        wheels = self.alvik.get_wheels_position('rad')
        dl, dr = ((p - q) * .0169 for p, q in zip(wheels, self.last_wheels))
        self.distance += (dl + dr) / 2
        # Integrate actual body rotation; tyres can slip during wheel turns.
        self.yaw = wrap(self.yaw + math.radians(self.alvik.get_gyros()[2]) * dt)
        self.last_wheels = wheels

    def turn_to(self, yaw, after):
        self.target_yaw, self.after_turn = wrap(yaw), after
        self.enter('TURN', 12.)

    def depart(self):
        sensed = label(self.alvik.get_color())
        if sensed not in COLOURS:
            self.fail('Cannot identify the departure line')
            return
        self.route = self.current = sensed
        if self.destination == self.current:
            self.enter('DOCKED', 1e9)
            self.note('Already at the selected colour')
            return
        self.route_plan = [self.current, 'CENTRAL', self.destination]
        self.note('Plan: ' + ' -> '.join(self.route_plan))
        self.armed = False
        if self.inbound:
            self.enter('INBOUND')
        else:
            self.turn_to(self.yaw + math.pi, 'INBOUND')

    def fail(self, reason):
        self.error = reason
        self.alvik.brake()
        self.enter('FAULT', 1e9)
        self.note(reason + ' | reload the world to reset')

    def pause(self):
        if self.state not in ('PAUSED', 'DOCKED', 'FAULT'):
            self.resume_state = self.state
            self.remaining = max(0., self.deadline - self.time)
            self.state = 'PAUSED'
            self.note('Paused | X resumes')
        self.alvik.brake()

    def resume(self):
        if self.state == 'PAUSED':
            self.state = self.resume_state
            self.deadline = self.time + self.remaining
            self.lost_since = self.marker_since = None
            self.note('Resumed ' + self.state)

    def follow(self):
        hits = [max(0, value - 130) for value in self.alvik.get_line_sensors()]
        if sum(hits) > 25:
            self.last_error = (hits[0] - hits[2]) / sum(hits)
            self.lost_since = None
            self.alvik.drive(5.0, 65 * self.last_error)
        else:
            if self.lost_since is None:
                self.lost_since = self.time
            if self.time - self.lost_since > 1.4:
                self.fail('Current line colour lost for 1.4 s')
            else:
                self.alvik.drive(2.0, 30 * self.last_error)

    def tick(self, dt):
        self._tick(dt)
        self.update_leds()

    def _tick(self, dt):
        self.time += dt
        self.sample_odometry(dt)
        ok_down, cancel_down = self.alvik.get_touch_ok(), self.alvik.get_touch_cancel()
        ok, cancel = ok_down and not self._ok_down, cancel_down and not self._cancel_down
        self._ok_down, self._cancel_down = ok_down, cancel_down
        if self.state == 'FAULT':
            self.alvik.brake()
            return
        if self.state == 'DOCKED':
            self.alvik.brake()
            if cancel:
                self.selected = COLOURS[(COLOURS.index(self.selected) + 1) % len(COLOURS)]
                self.note('Candidate ' + self.selected + ' (blinking); X starts')
            elif ok and not cancel_down:
                self.destination = self.selected
                self.depart()
            return
        if self.state == 'PAUSED':
            if ok and not cancel_down:
                self.resume()
            return
        if cancel_down or ok:
            self.pause()
            return
        if self.time > self.deadline:
            self.fail('Timeout in ' + self.state)
            return
        if self.state == 'TURN':
            error = wrap(self.target_yaw - self.yaw)
            if abs(error) < .012:
                self.alvik.brake()
                self.enter(self.after_turn)
            else:
                self.alvik.drive(0, max(-50, min(50, error * 90)))
            return
        if self.state == 'CENTER':
            if self.distance - self.phase_distance >= .097:
                self.alvik.brake()
                self.route = self.destination
                self.turn_to(BEARINGS[self.destination], 'EXIT')
            else:
                self.alvik.drive(2.5, 0)
            return
        if self.state == 'EXIT':
            if label(self.alvik.get_color()) == self.route and self.distance - self.phase_distance > .025:
                self.transfers.append(self.route)
                self.armed = False
                self.enter('OUTBOUND')
            elif self.distance - self.phase_distance > .18:
                self.fail('Could not acquire the requested exit colour')
            else:
                self.alvik.drive(2.5, 0)
            return
        colour = label(self.alvik.get_color())
        if colour == self.route:
            self.armed = True
        central = self.state == 'INBOUND' and self.armed and colour == 'BLACK'
        left, _, right = self.alvik.get_line_sensors()
        terminal = (self.state == 'OUTBOUND' and self.armed and colour == self.route
                    and min(left, right) > 100
                    and self.distance - self.phase_distance > .12)
        if central or terminal:
            if terminal:
                self.alvik.brake()
            else:
                self.alvik.drive(2., 0)
            if self.marker_since is None:
                self.marker_since = self.time
            if self.time - self.marker_since >= .06:
                if central:
                    self.yaw = wrap(BEARINGS[self.route] + math.pi)
                    self.enter('CENTER', 8.)
                else:
                    self.current = self.selected = self.route
                    self.inbound = False
                    self.arrivals.append(self.current)
                    self.enter('DOCKED', 1e9)
                    self.note('ARRIVED at ' + self.current + ' T; select the next destination')
            return
        self.marker_since = None
        self.follow()


if __name__ == "__main__":
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Colour Metro] Choose a coloured station and travel through the junction using known route bearings.')
    metro = Metro(alvik)
    previous = ticks_ms()
    while True:
        now = ticks_ms()
        metro.tick(ticks_diff(now, previous) / 1000.)
        previous = now
        sleep_ms(20)
