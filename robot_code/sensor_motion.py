"""Sensor and motion exercises: LEDs, ToF, odometry and a measured obstacle detour."""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

import math
from time import sleep_ms, ticks_add, ticks_diff, ticks_ms

from arduino_alvik import ArduinoAlvik

# Distances are centimetres; headings are degrees in the odometry frame.
TASKS = ('1 / LEDs', '2 / ToF distance', '3a / 20 cm straight', '3b / 30 cm square', '4 / 80 cm detour')
COLOURS = ((1, 0, 0), (0, 0, 1), (0, 1, 0), (1, 1, 1), (.65, 0, 1))
COLOUR_NAMES = ('red', 'blue', 'green', 'white', 'violet')
SQUARE = [(30, 0, 0), (30, 0, 90), (30, 30, 90), (30, 30, 180),
          (0, 30, 180), (0, 30, -90), (0, 0, -90), (0, 0, 0)]
BOX_DISTANCES = (8, 15, 25)


status = {}


def angle_error(target, heading):
    return (target - heading + 180) % 360 - 180


def go_to(alvik, target, pose):
    """Use encoder pose to reach the waypoint, then set its heading."""
    x, y, heading = pose
    tx, ty, final_heading = target
    dx, dy = tx - x, ty - y
    distance = math.sqrt(dx * dx + dy * dy)
    if distance > .28:
        error = angle_error(math.degrees(math.atan2(dy, dx)), heading)
        speed = min(8, max(1.2, distance * 2)) if abs(error) < 22 else 0
    else:
        error = angle_error(final_heading, heading)
        if abs(error) < 1.5:
            alvik.brake()
            return True
        speed = 0
    alvik.drive(speed, max(-55, min(55, 2.5 * error)))
    return False


def show_distance(alvik, values, now):
    left, center_left, _, center_right, right = values
    for led, distance in ((alvik.left_led, min(left, center_left)),
                          (alvik.right_led, min(center_right, right))):
        if distance < 10 and now // 250 % 2:
            led.set_color(0, 0, 0)
        elif distance > 15:
            led.set_color(0, 1, 0)
        else:
            led.set_color(1, 0, 0)


def main():
    alvik = ArduinoAlvik()
    alvik.begin()
    print('[Sensor Motion] Compare LED colours and distance readings, then try straight, square, and obstacle-detour motions.')
    task = colour = 0
    box_setting = 1
    running = True
    paused = False
    route = []
    held = set()
    result = ''
    next_report = ticks_ms()
    print('[SENSOR MOTION] A/D select exercise | W/S LEDs or box distance | X run/pause | C cancel | F reset')
    print('[SENSOR MOTION]', TASKS[task])
    while True:
        now = ticks_ms()
        pads = {name for name in ('ok', 'cancel', 'left', 'right', 'up', 'down', 'center')
                if getattr(alvik, 'get_touch_' + name)()}
        # Edge detection makes each held pad count as a single selection.
        pressed, held = pads - held, pads
        if 'cancel' in pads:
            alvik.brake()
            running = paused = False
            if 'cancel' in pressed:
                result = 'Cancelled'
        elif 'center' in pressed:
            alvik.brake()
            alvik.reset_pose(0, 0, 0)
            running, paused, result = task < 2, False, ''
        elif (task < 2 or not running) and ('left' in pressed or 'right' in pressed):
            task = (task + (1 if 'right' in pressed else -1)) % len(TASKS)
            running, paused, result = task < 2, False, ''
            alvik.brake()
            alvik.reset_pose(0, 0, 0)
            print('[SENSOR MOTION]', TASKS[task])
        elif task == 1 and ('up' in pressed or 'down' in pressed):
            box_setting = (box_setting + (1 if 'up' in pressed else -1)) % len(BOX_DISTANCES)
            running, paused, result = True, False, ''
            print('[SENSOR MOTION] Target distance:', BOX_DISTANCES[box_setting], 'cm; reposition the object for this measurement')
        elif 'ok' in pressed and task >= 2:
            if running:
                paused = not paused
                alvik.brake()
            else:
                running, paused, result = True, False, ''
                alvik.reset_pose(0, 0, 0)
                route = {2: [(20, 0, 0)], 3: list(SQUARE), 4: [(80, 0, 0)]}.get(task, [])
                detoured = False
        elif 'ok' in pressed and not running:
            running, result = True, ''
        if running and not paused:
            if task == 0:
                if 'up' in pressed or 'down' in pressed:
                    colour = (colour + (1 if 'up' in pressed else -1)) % len(COLOURS)
                    print('[SENSOR MOTION] LED', COLOUR_NAMES[colour])
                for led in (alvik.left_led, alvik.right_led):
                    led.set_color(*COLOURS[colour])
            elif task == 1:
                values = alvik.get_distance()
                show_distance(alvik, values, now)
                if ticks_diff(now, next_report) >= 0:
                    print('[SENSOR MOTION] L CL C CR R (cm):', ', '.join(f'{v:.1f}' for v in values))
                    next_report = ticks_add(now, 500)
            else:
                pose = alvik.get_pose()
                if task == 4 and not detoured:
                    ranges = alvik.get_distance()
                    front = min(ranges[1:4])
                    if front <= 14:
                        detoured = True
                        x = pose[0]
                        # The exercise box is 8 cm deep. The sensor is about 4 cm ahead
                        # of the axle. Allow room for the body to turn after the box.
                        past = x + front + 4 + 8 + 12
                        if past >= 80:
                            alvik.brake()
                            running = False
                            result = 'Move the box earlier on the course'
                            print('[SENSOR MOTION]', result)
                        else:
                            route = [(x, 0, 90), (x, 15, 0), (past, 15, -90),
                                     (past, 0, 0), (80, 0, 0)]
                            print('[SENSOR MOTION] DETOUR, front cm:', front)
                if running and go_to(alvik, route[0], pose):
                    route.pop(0)
                    if not route:
                        running = False
                        x, y, yaw = alvik.get_pose()
                        result = f'Done | Odom x {x:.1f}, y {y:.1f} cm, heading {angle_error(yaw, 0):.1f} deg'
                        print('[SENSOR MOTION]', TASKS[task], result)
        if result:
            description = result
        elif task == 0:
            description = COLOUR_NAMES[colour].capitalize() + ' | W/S changes colour'
        elif task == 1:
            description = f'Target distance {BOX_DISTANCES[box_setting]} cm'
        else:
            description = 'Paused' if paused else 'Running' if running else 'Ready'
        status.update(task=task, box_distance=BOX_DISTANCES[box_setting], running=running,
                      result=result, title=TASKS[task] + ' | ' + description,
                      hint='A/D exercise | X run/pause | C stop | F reset')
        sleep_ms(20)


if __name__ == '__main__':
    main()
