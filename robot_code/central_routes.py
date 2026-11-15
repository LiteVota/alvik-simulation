
"""Select a colour at Central, follow it to its stop, then return to Central."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

from time import sleep_ms, ticks_diff, ticks_ms
import math

from arduino_alvik import ArduinoAlvik

LED_COLOURS = {
    'ORANGE': (1, .42, 0),
    'MAGENTA': (1, 0, .55),
    'CYAN': (0, 1, 1),
}
COLOURS = tuple(LED_COLOURS)


def classify(rgb):
    h, s, v = ArduinoAlvik.rgb2hsv(*rgb)
    if v < .34:
        return 'BLACK'
    if s < .24:
        return 'MARKER' if v < .68 else 'WHITE'
    hue = h  # Arduino returns hue in degrees.
    if 21 <= hue <= 58:
        return 'ORANGE'
    if 280 <= hue <= 350:
        return 'MAGENTA'
    if 155 <= hue <= 207:
        return 'CYAN'
    return 'OTHER'


def wrap(angle):
    return (angle + 180) % 360 - 180


class CentralRoutes:
    def __init__(self, alvik):
        self.alvik = alvik
        self.selected = 'ORANGE'
        self.current = 'CENTRAL'
        self.time = self.distance = self.heading = 0.
        self.wheels = alvik.get_wheels_position('rad')
        self.ok_down = self.cancel_down = self.paused = False
        self.enter('DOCKED', math.inf)

    def enter(self, state, timeout=55.):
        self.state = state
        self.start_distance = self.distance
        self.deadline = self.time + timeout
        self.last_seen = self.time
        self.last_error = 0.
        self.marker_samples = 0
        print(f'[ROUTES] {state} | {self.selected}')

    def turn(self, heading, next_state):
        self.target_heading, self.next_state = heading, next_state
        self.enter('TURN', 9.)

    def fault(self, reason):
        self.alvik.brake()
        self.enter('STOPPED', math.inf)
        print(f'[ROUTES] {reason}. Reload the world to reset.')

    def sample_motion(self, dt):
        wheels = self.alvik.get_wheels_position('rad')
        self.distance += .0169 * sum(p - q for p, q in zip(wheels, self.wheels)) / 2
        self.wheels = wheels
        self.heading = wrap(self.heading + self.alvik.get_gyros()[2] * dt)

    def follow(self, colour, speed, next_state, hub=False):
        left, center, right = self.alvik.get_line_sensors()
        # The wide junction disk reaches both outer IR sensors. At a terminal
        # the single colour sensor reads the transverse grey marker.
        marker = min(left, right) > 160 if hub else classify(self.alvik.get_color()) == 'MARKER'
        self.marker_samples = self.marker_samples + 1 if (
            marker and self.distance - self.start_distance > .065) else 0
        if self.marker_samples:
            self.alvik.drive(2.5, 0)
            if self.marker_samples >= 3:
                self.alvik.brake()
                self.hub_heading = self.heading
                self.enter(next_state)
            return
        hits = [max(0, value - 130) for value in (left, center, right)]
        if sum(hits) > 25:
            self.last_seen = self.time
            self.last_error = (hits[0] - hits[2]) / sum(hits)
            self.alvik.drive(speed, 74 * self.last_error)
        elif self.time - self.last_seen < 1.25:
            self.alvik.drive(2.5, 46 * self.last_error)
        else:
            self.fault(f'Lost the {colour.lower()} tape in {self.state}')

    def start_scan(self, colour, next_state):
        self.scan_colour, self.scan_next = colour, next_state
        self.scan_armed, self.scan_edge = False, None
        self.enter('SCAN', 13.)
        print(f'[ROUTES] Looking for {colour} with the floor colour sensor')

    def scan(self):
        matching = classify(self.alvik.get_color()) == self.scan_colour
        if not matching:
            if self.scan_edge is not None:
                span = wrap(self.heading - self.scan_edge)
                if span >= 3.:
                    centre_heading = wrap(self.scan_edge + span / 2.)
                    print(f'[ROUTES] Sensor found {self.scan_colour} at {centre_heading:+.1f} deg')
                    self.turn(centre_heading, self.scan_next)
                    return
                self.scan_edge = None
            self.scan_armed = True
        elif self.scan_armed and self.scan_edge is None:
            self.scan_edge = self.heading
        self.alvik.drive(0, 46.)

    def straight(self, distance_cm, heading):
        error = wrap(heading - self.heading)
        if self.distance - self.start_distance >= distance_cm / 100:
            self.alvik.brake()
            return True
        self.alvik.drive(4.2, max(-42, min(42, 3 * error)))
        return False

    def tick(self, dt):
        self.sample_motion(dt)
        ok, cancel = self.alvik.get_touch_ok(), self.alvik.get_touch_cancel()
        confirm, cycle = ok and not self.ok_down, cancel and not self.cancel_down
        self.ok_down, self.cancel_down = ok, cancel
        state = self.state
        if state == 'STOPPED':
            return
        if state == 'DOCKED':
            self.time += dt
            if cycle:
                self.selected = COLOURS[(COLOURS.index(self.selected) + 1) % len(COLOURS)]
                print(f'[ROUTES] Selected {self.selected}')
            elif confirm and not cancel:
                self.enter('TO_HUB')
            return
        if cycle or (confirm and not cancel):
            self.paused = cycle or not self.paused
            if self.paused:
                self.alvik.brake()
                print('[ROUTES] Paused | X resumes')
            else:
                print('[ROUTES] Resumed')
            return
        if self.paused:
            return
        self.time += dt
        if self.time > self.deadline:
            self.fault(f'Timeout in {state}')
        elif state == 'TO_HUB':
            self.follow('BLACK', 6.5, 'CENTER_OUT', hub=True)
        elif state == 'CENTER_OUT':
            if self.straight(11.35, self.hub_heading):
                self.start_scan(self.selected, 'OUTBOUND')
        elif state == 'SCAN':
            self.scan()
        elif state == 'TURN':
            error = wrap(self.target_heading - self.heading)
            if abs(error) < .65:
                self.alvik.brake()
                self.enter(self.next_state)
            else:
                self.alvik.drive(0, max(-64, min(64, 2.8 * error)))
        elif state == 'OUTBOUND':
            self.follow(self.selected, 8., 'TERMINAL')
        elif state == 'TERMINAL':
            self.current = self.selected
            print(f'[ROUTES] ARRIVED {self.selected}')
            self.turn(wrap(self.heading + 180.), 'INBOUND')
        elif state == 'INBOUND':
            self.follow(self.selected, 8., 'CENTER_HOME', hub=True)
        elif state == 'CENTER_HOME':
            if self.straight(11.35, self.hub_heading):
                self.start_scan('BLACK', 'RETURN_TO_DOCK')
        elif state == 'RETURN_TO_DOCK':
            self.follow('BLACK', 5., 'HOME_MARKER')
        elif state == 'HOME_MARKER':
            self.turn(wrap(self.heading + 180.), 'PARK')
        elif state == 'PARK':
            self.current = 'CENTRAL'
            print(f'[ROUTES] RETURNED CENTRAL from {self.selected}')
            self.enter('DOCKED', math.inf)

    def show(self):
        colour = LED_COLOURS[self.selected]
        if self.state == 'DOCKED' and int(self.time / .28) % 2:
            colour = (0, 0, 0)
        elif self.state == 'STOPPED':
            colour = (1, 0, 0)
        for led in (self.alvik.left_led, self.alvik.right_led):
            led.set_color(*colour)


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Central Routes] Select a route colour; Alvik finds its tape, visits the destination, and returns to Central.')
    routes = CentralRoutes(alvik)
    print('Central Routes | C cycles colour while docked | X starts or pauses/resumes')
    previous = ticks_ms()
    while True:
        now = ticks_ms()
        routes.tick(ticks_diff(now, previous) / 1000.)
        previous = now
        routes.show()
        sleep_ms(20)


if __name__ == '__main__':
    main()
