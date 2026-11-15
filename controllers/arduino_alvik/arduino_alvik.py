"""Alvik bridge: API implementation and optional Webots scene tools.

1. Robot API: units, device access, motion, sensors, LEDs, touch input, and time.
2. Webots displays: distance arrays, movable targets, experiment resets, and HUDs.
3. Scene controllers: Blank Template presets and source reload in editable worlds.
4. Launcher: choose a robot program and attach the requested Webots tools.

ALVIK.controllerArgs accepts a robot_code filename followed by optional flags:
  <program.py>       Run the portable Alvik behavior file.
  --beams            Accepted for compatibility; V toggles sensors in every program.
                     The guide is created automatically and starts hidden.
  --hand-target      HandTarget: J nearer, K farther, plus a range HUD.
                     Creates a target ahead of Alvik when TARGET is absent.
  --status           ProgramStatus: show the program's status title and hint.
  --lab              LabTools: F/start resets and experiment labels; optional LAB_BOX.
                     May accompany display flags above.
All four feature flags can be combined with any robot program. --lab creates
its exercise box when required, or provides F restart for other programs.
Scene tools use supervisor TRUE, the default in the shared Alvik model.
V toggles sensor readings and the guide in every program, without any flags.

A separate scene-helper Robot uses one standalone argument (no program filename):
  --playground-tools  Playground HUD; U or saving Python reloads the active program.
                     Reads the ALVIK node and its controllerArgs/customData.
  --blank-tools       Blank Template HUD, 1-4 scene presets, U/save code reload.
                     Requires ALVIK, TEMPLATE_LAYOUT, and the six PROP_* nodes.

X/C and directional keyboard mappings in the API emulate Alvik touch pads; each
robot program decides what those inputs do. HUDs, object placement, and resets
remain here. Physical Alvik uses Arduino's own library instead of this bridge.
"""

# SPDX-License-Identifier: MPL-2.0
# See THIRD_PARTY_NOTICES.md for upstream sources and retained notices.

import math
from pathlib import Path
import sys
import time

from controller import Robot, Supervisor

_robot_class = Robot
_setup_world = ()
PROGRAM_NAMESPACE = {}
SOURCE_ROOT = Path(__file__).resolve().parents[2] / "robot_code"

# 1. Robot API: hardware-compatible methods implemented with Webots devices.

WHEEL_RADIUS_M = 0.0169
WHEEL_TRACK_M = 0.0895
TOF_MAX_M = 3.5
# 4x4 carrier ROI averages, indexed from the upper-left of the rendered image.
TOF_REGIONS = ((4,), (5,), (5, 6), (6,), (7,))
TOF_TOP = (0, 1, 2, 3)
TOF_BOTTOM = (12, 13, 14)
_DISTANCE = {"m": 1., "cm": .01, "mm": .001, "inch": .0254, "in": .0254}
_ANGLE = {"rad": 1., "deg": math.pi / 180, "rev": 2 * math.pi,
          "revolution": 2 * math.pi}
_SPEED = {"rad/s": 1., "deg/s": math.pi / 180,
          "rpm": math.pi / 30, "rev/s": 2 * math.pi}


def factor(table, unit):
    try:
        return table[unit]
    except KeyError:
        raise ValueError("Unsupported unit: " + str(unit)) from None


def rgb2hsv(r, g, b):
    """Convert normalized RGB to HSV with hue in degrees."""
    min_ = min(r, g, b)
    max_ = max(r, g, b)

    v = max_
    delta = max_ - min_

    if delta < 0.00001:
        h = 0
        s = 0
        return h, s, v

    if max_ > 0:
        s = delta / max_
    else:
        s = 0
        h = None
        return h, s, v

    if r >= max_:
        h = (g - b) / delta  # color is between yellow and magenta
    elif g >= max_:
        h = 2.0 + (b - r) / delta
    else:
        h = 4.0 + (r - g) / delta

    h *= 60.0
    if h < 0:
        h += 360.0

    return h, s, v


def hsv2label(h, s, v):
    """Classify HSV using Arduino's published colour-label thresholds."""

    if None in [h, s, v]:
        return 'UNDEFINED'

    if s < 0.1:
        if v < 0.05:
            label = 'BLACK'
        elif v < 0.15:
            label = 'GREY'
        elif v < 0.8:
            label = 'LIGHT GREY'
        else:
            label = 'WHITE'
    else:
        if v > 0.1:
            if 20 <= h < 90:
                label = 'YELLOW'
            elif 90 <= h < 140:
                label = 'LIGHT GREEN'
            elif 140 <= h < 170:
                label = 'GREEN'
            elif 170 <= h < 210:
                label = 'LIGHT BLUE'
            elif 210 <= h < 250:
                label = 'BLUE'
            elif 250 <= h < 280:
                label = 'VIOLET'
            else:  # h<20 or h>=280 is more problematic
                if v < 0.5 and s < 0.45:
                    label = 'BROWN'
                else:
                    if v > 0.77:
                        label = 'ORANGE'
                    else:
                        label = 'RED'
        else:
            label = 'BLACK'
    return label


GROUND_WHITE = 850.0
GROUND_BLACK = 300.0
_active = None
_use_lab_tools = False


class SimulationEnded(BaseException):
    """Normal Webots termination, including Reload World."""


class _LED:
    """Arduino-style on/off RGB channels backed by a Webots RGB LED."""
    def __init__(self, device):
        self.device = device
        self.color = (0, 0, 0)

    def set_color(self, red, green, blue):
        if not all(math.isfinite(v) and 0 <= v <= 1 for v in (red, green, blue)):
            raise ValueError('LED channels must be finite values in 0..1')
        self.color = tuple(int(bool(channel)) for channel in (red, green, blue))
        r, g, b = (round(255*v) for v in self.color)
        self.device.set((r << 16) | (g << 8) | b)


class ArduinoAlvik:
    def __init__(self):
        global _active
        if _active is not None:
            raise RuntimeError("This starter supports one ArduinoAlvik instance")
        self.robot = _robot_class()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motors = [self._device(s + " wheel motor") for s in ("left", "right")]
        self.ground = [self._device("gs" + str(i)) for i in range(3)]
        self.keyboard = self.robot.getKeyboard()
        self.left_led = _LED(self._device('left led'))
        self.right_led = _LED(self._device('right led'))
        self.DL1, self.DL2 = self.left_led, self.right_led
        self._keys = set()
        self._new_keys = set()
        self._ok_pending = False
        self._cancel_pending = False
        self._ended = False
        self._begun = False
        self._displays = []
        self._experiment_tools = None
        self._setup_extensions()
        self._setup_colour()
        _active = self

    def _setup_extensions(self):
        if hasattr(self.robot, "getNumberOfDevices"):
            inventory = {self.robot.getDeviceByIndex(i).getName(): self.robot.getDeviceByIndex(i)
                         for i in range(self.robot.getNumberOfDevices())}
            self._optional = inventory.get
        else:
            self._optional = self.robot.getDevice  # Test doubles.
        self.encoders = [self._optional(side + " wheel sensor") for side in ("left", "right")]
        self.tof = self._device("front ToF")
        self._tof_period = math.ceil(1000 / (30 * self.timestep)) * self.timestep
        self.accelerometer = self._optional("accelerometer")
        self.gyro = self._optional("gyro")
        self._encoder_previous = None
        self._encoder_angles = None
        self._wheel_rad_s = None
        self._pose = [0., 0., 0.]

    def _enable_extensions(self):
        self.tof.enable(self._tof_period)
        for device in self.encoders + [self.accelerometer, self.gyro]:
            if device is not None:
                device.enable(self.timestep)

    def _sample_encoders(self):
        if any(sensor is None for sensor in self.encoders):
            return
        angles = tuple(sensor.getValue() for sensor in self.encoders)
        if not all(math.isfinite(v) for v in angles):
            raise RuntimeError("Wheel encoders are not ready or are invalid")
        self._encoder_angles = angles
        if self._encoder_previous is None:
            self._encoder_previous = angles
            return  # A speed estimate needs two samples, not an invented zero.
        increments = tuple(alvik - b for alvik, b in zip(angles, self._encoder_previous))
        self._encoder_previous = angles
        self._wheel_rad_s = tuple(v / (self.timestep / 1000) for v in increments)
        dl, dr = (v * WHEEL_RADIUS_M for v in increments)
        distance, turn = (dl + dr) / 2, (dr - dl) / WHEEL_TRACK_M
        heading = self._pose[2]
        if abs(turn) < 1e-10:
            self._pose[0] += distance * math.cos(heading)
            self._pose[1] += distance * math.sin(heading)
        else:
            radius = distance / turn
            self._pose[0] += radius * (math.sin(heading + turn) - math.sin(heading))
            self._pose[1] += radius * (math.cos(heading) - math.cos(heading + turn))
        self._pose[2] += turn

    def get_distance(self, unit="cm"):
        return self._range_regions(TOF_REGIONS, unit)

    def get_distance_top(self, unit="cm"):
        return self._range_regions((TOF_TOP,), unit)[0]

    def get_distance_bottom(self, unit="cm"):
        return self._range_regions((TOF_BOTTOM,), unit)[0]

    def _range_regions(self, regions, unit, image=None):
        if image is None:
            image = self.tof.getRangeImage()
        if len(image) != 16 or any(math.isnan(value) or value < 0 for value in image):
            raise RuntimeError("Invalid 4x4 ToF frame")
        meters_per_unit = factor(_DISTANCE, unit)
        return tuple(sum(min(image[i], TOF_MAX_M) for i in zone) /
                     (len(zone) * meters_per_unit) for zone in regions)

    def get_wheels_position(self, unit="deg"):
        if self._encoder_angles is None:
            raise RuntimeError("Wheel encoders missing or not sampled")
        return tuple(v / factor(_ANGLE, unit) for v in self._encoder_angles)

    def get_wheels_speed(self, unit="rpm"):
        if self._wheel_rad_s is None:
            raise RuntimeError("Measured wheel speed needs two encoder samples; call sleep_ms first")
        return tuple(v / factor(_SPEED, unit) for v in self._wheel_rad_s)

    def drive(self, linear_velocity, angular_velocity, linear_unit="cm/s", angular_unit="deg/s"):
        if not linear_unit.endswith("/s"):
            raise ValueError("Linear velocity unit must end in /s")
        v = linear_velocity * factor(_DISTANCE, linear_unit[:-2])
        w = angular_velocity * factor(_SPEED, angular_unit)
        if not all(math.isfinite(value) for value in (v, w)):
            raise ValueError("Drive velocities must be finite")
        self.set_wheels_speed((v - w * WHEEL_TRACK_M / 2) / WHEEL_RADIUS_M,
                              (v + w * WHEEL_TRACK_M / 2) / WHEEL_RADIUS_M, unit="rad/s")

    def get_drive_speed(self, linear_unit="cm/s", angular_unit="deg/s"):
        if not linear_unit.endswith("/s"):
            raise ValueError("Linear velocity unit must end in /s")
        left, right = self.get_wheels_speed("rad/s")
        v = WHEEL_RADIUS_M * (left + right) / 2
        w = WHEEL_RADIUS_M * (right - left) / WHEEL_TRACK_M
        return v / factor(_DISTANCE, linear_unit[:-2]), w / factor(_SPEED, angular_unit)

    def get_pose(self, distance_unit="cm", angle_unit="deg"):
        if self._encoder_angles is None:
            raise RuntimeError("Odometry needs wheel encoders")
        return (self._pose[0] / factor(_DISTANCE, distance_unit),
                self._pose[1] / factor(_DISTANCE, distance_unit),
                self._pose[2] / factor(_ANGLE, angle_unit))

    def reset_pose(self, x=0, y=0, theta=0, distance_unit="cm", angle_unit="deg"):
        values = [x * factor(_DISTANCE, distance_unit), y * factor(_DISTANCE, distance_unit),
                  theta * factor(_ANGLE, angle_unit)]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Pose must be finite")
        self._pose = values  # Reset estimate, never teleport the simulated robot.

    def _touch(self, letter):
        codes = {ord(letter.upper()), ord(letter.lower())}
        return bool(self._keys.intersection(codes))

    def get_touch_up(self): return self._touch("W")
    def get_touch_down(self): return self._touch("S")
    def get_touch_left(self): return self._touch("A")
    def get_touch_right(self): return self._touch("D")
    def get_touch_center(self): return self._touch("F")

    def get_touch_any(self):
        return (self._ok_pending or self._cancel_pending or
                self._touch("X") or self._touch("C") or
                any((self.get_touch_up(), self.get_touch_down(), self.get_touch_left(),
                     self.get_touch_right(), self.get_touch_center())))

    def _get_line_raw(self):
        """Simulator diagnostic, not an asserted Arduino API call."""
        return tuple(sensor.getValue() for sensor in self.ground)

    def get_accelerations(self):
        values = self.accelerometer.getValues()
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Invalid accelerometer sample")
        return tuple(value / 9.80665 for value in values)

    def get_gyros(self):
        values = self.gyro.getValues()
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Invalid gyro sample")
        return tuple(map(math.degrees, values))

    def get_imu(self):
        return self.get_accelerations() + self.get_gyros()

    def _setup_colour(self):
        self.colour_camera = self._device('colour sensor')
        self._colour_enabled = False
        self._colour_black = (0., 0., 0.)
        self._colour_white = (1., 1., 1.)

    def _enable_colour_sensor(self):
        """Start sampling the one downward colour sensor."""
        if not self._colour_enabled:
            self.colour_camera.enable(self.timestep)
            self._colour_enabled = True
            self._step()

    def _read_rgb(self):
        self._enable_colour_sensor()
        camera = self.colour_camera
        image = camera.getImage()
        if image is None:
            raise RuntimeError('Colour camera has no rendered sample yet')
        width, height = camera.getWidth(), camera.getHeight()
        # Tiny cameras average their full optical footprint (not world data).
        channels = (camera.imageGetRed, camera.imageGetGreen, camera.imageGetBlue)
        return tuple(sum(channel(image, width, x, y)
                         for x in range(width) for y in range(height)) /
                     (255 * width * height) for channel in channels)

    def get_color(self, color_format='rgb'):
        raw = self._read_rgb()
        rgb = tuple(max(0., min(1., (v - low) / (high - low)))
                    for v, low, high in zip(raw, self._colour_black, self._colour_white))
        if color_format == 'rgb':
            return rgb
        if color_format == 'hsv':
            return rgb2hsv(*rgb)
        raise ValueError("color_format must be 'rgb' or 'hsv'")

    def get_color_label(self):
        return hsv2label(*self.get_color('hsv'))

    def color_calibration(self, background='white'):
        if background not in ('white', 'black'):
            raise ValueError("background must be 'white' or 'black'")
        sample = self._read_rgb()
        low = sample if background == 'black' else self._colour_black
        high = sample if background == 'white' else self._colour_white
        if any(h - l < .1 for h, l in zip(high, low)):
            raise ValueError('Calibration needs distinct black and white samples')
        self._colour_black, self._colour_white = low, high

    @staticmethod
    def rgb2hsv(r, g, b):
        return rgb2hsv(r, g, b)

    @staticmethod
    def hsv2label(h, s, v):
        return hsv2label(h, s, v)

    def _device(self, name):
        device = self.robot.getDevice(name)
        if device is None:
            raise RuntimeError("Missing Webots device: " + name)
        return device

    def begin(self):
        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0)
        for sensor in self.ground:
            sensor.enable(self.timestep)
        self.keyboard.enable(self.timestep)
        self._enable_extensions()
        self._begun = True
        # Wait for one range frame and at least two wheel encoder samples.
        for _ in range(max(2, self._tof_period // self.timestep)):
            self._step()
        # Optional displays and reset helpers are implemented in section 2.
        display_types = (RangeOverlay,) + tuple(display for display in _setup_world if display is not RangeOverlay)
        self._displays = [display(self) for display in display_types]
        if _use_lab_tools:
            self._experiment_tools = LabTools(self)
        print("Alvik ready. Click the 3D view. X = OK, C = Cancel, V = sensor array (initially hidden).", flush=True)
        return 0

    def _step(self):
        if not self._begun:
            raise RuntimeError("Call alvik.begin() before advancing the simulation")
        if self._ended or self.robot.step(self.timestep) == -1:
            self._ended = True
            raise SimulationEnded()
        self._sample_encoders()
        previous_keys = self._keys
        self._keys = set()
        key = self.keyboard.getKey()
        while key != -1:
            self._keys.add(key & 0xFFFF)
            key = self.keyboard.getKey()
        new_keys = self._keys - previous_keys
        self._new_keys = new_keys
        if new_keys.intersection((ord('X'), ord('x'))):
            self._ok_pending = True
        if new_keys.intersection((ord('C'), ord('c'))):
            self._cancel_pending = True
            self._ok_pending = False
            self.brake()
        if self._experiment_tools is not None:
            self._experiment_tools.update()
        status = PROGRAM_NAMESPACE.get('status', {})
        if status.get('title'):
            self.robot.setCustomData(status['title'])
        for display in self._displays:
            display.update()

    def get_touch_ok(self):
        pressed = self._ok_pending or self._touch("X")
        self._ok_pending = False
        return pressed

    def get_touch_cancel(self):
        pressed = self._cancel_pending or self._touch("C")
        self._cancel_pending = False
        return pressed

    def get_line_sensors(self):
        # Same ordering as Alvik: left, centre, right. Black high, white low.
        raw = self._get_line_raw()
        if not all(math.isfinite(value) for value in raw):
            raise RuntimeError("Invalid ground sensor reading")
        return tuple(round(max(0, min(1000, (GROUND_WHITE - value) *
                        1000 / (GROUND_WHITE - GROUND_BLACK)))) for value in raw)

    def set_wheels_speed(self, left_speed, right_speed, unit="rpm"):
        conversion = factor(_SPEED, unit)
        if not all(math.isfinite(speed) for speed in (left_speed, right_speed)):
            raise ValueError("Wheel speed must be finite")
        for motor, speed in zip(self.motors, (left_speed, right_speed)):
            limit = motor.getMaxVelocity()
            motor.setVelocity(max(-limit, min(limit, speed * conversion)))

    def move(self, distance, unit="cm", blocking=True):
        """Blocking straight movement with wheel encoder feedback."""
        if not blocking:
            raise NotImplementedError("move(blocking=False) is not implemented by this adapter")
        target = distance * factor(_DISTANCE, unit)
        start = self.get_wheels_position("rad")
        self._relative_motion(target, start=start)

    def rotate(self, angle, unit="deg", blocking=True):
        """Blocking rotation with simulated gyro feedback, positive CCW."""
        if not blocking:
            raise NotImplementedError("rotate(blocking=False) is not implemented by this adapter")
        target = angle * factor(_ANGLE, unit)
        self._relative_motion(target)

    def _relative_motion(self, target, start=None):
        if not math.isfinite(target):
            raise ValueError('Movement must be finite')
        dt = self.timestep / 1000
        elapsed, travelled = 0., 0.
        paused = False
        timeout = 4 + abs(target) / (.035 if start is not None else .55)
        try:
            while elapsed < timeout:
                if self.get_touch_cancel():
                    return False
                if self._ok_pending:
                    self._ok_pending = False
                    paused = not paused
                error = target - travelled
                if paused:
                    self.brake()
                elif start is not None:
                    if abs(error) < .0015:
                        return True
                    self.drive(max(-6, min(6, 430 * error)), 0)
                else:
                    if abs(error) < .012:
                        return True
                    self.drive(0, max(-60, min(60, 140 * error)))
                self._step()
                if not paused:
                    elapsed += dt
                if start is not None:
                    travelled = sum(p - q for p, q in zip(self.get_wheels_position("rad"), start))
                    travelled *= WHEEL_RADIUS_M / 2
                else:
                    travelled += math.radians(self.get_gyros()[2]) * dt
            print('[ALVIK] Movement timed out', flush=True)
            return False
        finally:
            self.brake()

    def brake(self):
        if not self._ended:
            self.set_wheels_speed(0, 0)

    def stop(self):
        self.brake()


def sleep_ms(milliseconds):
    if _active is None or not _active._begun:
        raise RuntimeError("Call alvik.begin() before sleep_ms()")
    if milliseconds < 0:
        raise ValueError("sleep_ms requires a nonnegative duration")
    # Durations round up to whole physics steps in the active world.
    steps = max(1, math.ceil(milliseconds / _active.timestep))
    for _ in range(steps):
        _active._step()


# MicroPython tick arithmetic uses a power-of-two wrap interval.
# The interval here is an adapter convention, not a guarantee for every port.
TICKS_PERIOD = 1 << 30

def sleep(seconds):
    sleep_ms(math.ceil(seconds * 1000))

def _now():
    if _active is None or not _active._begun:
        raise RuntimeError("Simulation time requires alvik.begin() first")
    return _active.robot.getTime()

def ticks_ms(): return round(_now() * 1000) % TICKS_PERIOD
def ticks_us(): return round(_now() * 1000000) % TICKS_PERIOD
def ticks_diff(new, old):
    return ((new - old + TICKS_PERIOD // 2) % TICKS_PERIOD) - TICKS_PERIOD // 2
def ticks_add(ticks, delta): return (ticks + delta) % TICKS_PERIOD

FUNCTIONS = dict(time=_now, sleep=sleep, sleep_ms=sleep_ms, ticks_ms=ticks_ms,
                 ticks_us=ticks_us, ticks_diff=ticks_diff, ticks_add=ticks_add)


# 2. Webots-only displays and experiment scene helpers.
# Hooked into begin()/_step(); no extra Alvik methods are exposed to robot code.


def scene_node(robot, name, text):
    """Reuse a named scene object or create it from built-in Webots nodes."""
    node = robot.getFromDef(name)
    if node is None:
        robot.getRoot().getField('children').importMFNodeFromString(-1, text)
        node = robot.getFromDef(name)
    if node is None:
        raise RuntimeError('Could not create scene object ' + name + '; use supervisor TRUE.')
    return node


def rotation_axis_angle(matrix):
    """Convert a world rotation matrix to Webots axis-angle, including 180 deg."""
    trace = matrix[0] + matrix[4] + matrix[8]
    if trace > 0:
        scale = 2 * math.sqrt(trace + 1)
        scalar = scale / 4
        vector = [(matrix[7] - matrix[5]) / scale,
                  (matrix[2] - matrix[6]) / scale,
                  (matrix[3] - matrix[1]) / scale]
    else:
        i = max(range(3), key=lambda axis: matrix[4 * axis])
        j, k = (i + 1) % 3, (i + 2) % 3
        scale = 2 * math.sqrt(max(0., 1 + matrix[4*i] - matrix[4*j] - matrix[4*k]))
        scalar = (matrix[3*k+j] - matrix[3*j+k]) / scale
        vector = [0., 0., 0.]
        vector[i] = scale / 4
        vector[j] = (matrix[3*j+i] + matrix[3*i+j]) / scale
        vector[k] = (matrix[3*k+i] + matrix[3*i+k]) / scale
    norm = math.sqrt(sum(value * value for value in vector))
    if norm < 1e-10:
        return [0., 0., 1., 0.]
    return [*(value / norm for value in vector), 2 * math.atan2(norm, scalar)]


class RangeOverlay:
    """V toggles a complete, static line mesh using viewpoint visibility.

    The geometry is built once at startup. Toggling never edits its indices or
    points, and motion updates only the guide pose. The guide is always hidden
    from the range finder and has no collision geometry.
    """

    def __init__(self, alvik):
        self.alvik, self.robot = alvik, alvik.robot
        self.sensor = self.robot.getSelf().getFromProtoDef('FRONT_TOF')
        self.width, self.height = alvik.tof.getWidth(), alvik.tof.getHeight()
        self.visible, self.last = False, -1.
        vertices, segments = [(0., 0., 0.)], []
        depth = .18
        half_width = depth * math.tan(alvik.tof.getFov() / 2)
        half_height = half_width * self.height / self.width
        for col in range(self.width + 1):
            y = half_width * (1 - 2 * col / self.width)
            index = len(vertices)
            vertices.extend(((depth, y, half_height), (depth, y, -half_height)))
            segments.extend((index, index + 1, -1))
            if col in (0, self.width):
                segments.extend((0, index, -1, 0, index + 1, -1))
        for row in range(self.height + 1):
            z = half_height * (1 - 2 * row / self.height)
            index = len(vertices)
            vertices.extend(((depth, half_width, z), (depth, -half_width, z)))
            segments.extend((index, index + 1, -1))
        point_text = ', '.join(' '.join(f'{value:.9g}' for value in point) for point in vertices)
        index_text = ' '.join(map(str, segments))
        old = self.robot.getFromDef('ALVIK_SENSOR_GUIDE')
        pose = self.robot.getFromDef('ALVIK_SENSOR_POSE')
        if old is not None and pose is None:
            # Replace a guide left by the earlier incremental-index implementation.
            old.remove()
        self.overlay = scene_node(self.robot, 'ALVIK_SENSOR_GUIDE', f'''
            DEF ALVIK_SENSOR_GUIDE Solid {{
              name "Alvik sensor guide" locked TRUE
              children [ DEF ALVIK_SENSOR_POSE Pose {{
                translation 0 0 -1000
                children [ Shape {{
                  appearance Appearance {{ material Material {{ emissiveColor 1 .03 .01 }} }}
                  geometry DEF ALVIK_SENSOR_LINES IndexedLineSet {{
                    coord DEF ALVIK_SENSOR_POINTS Coordinate {{ point [ {point_text} ] }}
                    coordIndex [ {index_text} ]
                  }}
                }} ]
              }} ]
            }}''')
        pose = self.robot.getFromDef('ALVIK_SENSOR_POSE')
        self.translation, self.rotation = pose.getField('translation'), pose.getField('rotation')
        children = self.robot.getRoot().getField('children')
        self.viewpoints = [children.getMFNode(index) for index in range(children.getCount())
                           if children.getMFNode(index).getTypeName() == 'Viewpoint']
        self.overlay.setVisibility(self.sensor, False)
        self._set_visible(False)
        self._controls_hint()

    def _sensor_top(self):
        return .225 if HandTarget in _setup_world else .19

    def _controls_hint(self):
        action = 'hide' if self.visible else 'show'
        self.robot.setLabel(50, f'V: {action} sensor array  |  X: OK  |  C: Cancel',
                            .025, self._sensor_top() - .045, .023, 0x203040, 0, 'Arial')

    def _set_visible(self, visible):
        for viewpoint in self.viewpoints:
            self.overlay.setVisibility(viewpoint, visible)

    def _move_guide(self):
        self.translation.setSFVec3f(self.sensor.getPosition())
        self.rotation.setSFRotation(rotation_axis_angle(self.sensor.getOrientation()))

    def update(self):
        if self.alvik._new_keys.intersection((ord('V'), ord('v'))):
            self.visible = not self.visible
            self._controls_hint()
            if self.visible:
                self._move_guide()
                self._set_visible(True)
                self.last = -1.
                angle = math.degrees(self.alvik.tof.getFov())
                self.robot.setLabel(1, f'{self.width} x {self.height} ToF | {angle:.0f} deg horizontal | guide at 18 cm',
                                    .025, self._sensor_top() + .04, .025, 0x203040, 0, 'Arial')
            else:
                self._set_visible(False)
                for label in (0, 1, 2, *range(10, 10 + self.height)):
                    self.robot.setLabel(label, '', 0, 0, .025, 0x203040, 0, 'Arial')
        if not self.visible:
            return
        now = self.robot.getTime()
        if now - self.last < .05 - 1e-9:
            return
        self.last = now
        self._move_guide()
        frame = self.alvik.tof.getRangeImage()
        values = self.alvik._range_regions(TOF_REGIONS + (TOF_TOP, TOF_BOTTOM), 'cm', frame)
        text = ' / '.join('MAX' if value >= 349.9 else f'{value:.1f}' for value in values)
        self.robot.setLabel(0, 'cm L / CL / C / CR / R / T / B: ' + text,
                            .025, self._sensor_top(), .03, 0x203040, 0, 'Arial')
        for row in range(self.height):
            cells = frame[row * self.width:(row + 1) * self.width]
            text = '  '.join(f'{value * 100:5.1f}' if value < TOF_MAX_M else '  MAX' for value in cells)
            self.robot.setLabel(10 + row, text, .025, self._sensor_top() + .08 + .025 * row, .022, 0x203040, 0, 'Arial')


class ProgramStatus:
    """--status: display ordinary program data without controlling the robot."""

    def __init__(self, alvik):
        self.alvik, self.robot = alvik, alvik.robot
        self.previous = None

    def update(self):
        # Scene helpers or lab tools already supply the program's top-left block.
        if _use_lab_tools or self.robot.getFromDef('SCENE_TOOLS') is not None:
            return
        status = PROGRAM_NAMESPACE.get('status', {})
        fallback = Path(PROGRAM_NAMESPACE.get('__file__', 'Alvik')).stem.replace('_', ' ').title()
        y = .025
        content = (status.get('title', fallback + ' | Running'), status.get('hint', 'V toggles the sensor array'), y)
        if content != self.previous:
            self.robot.setLabel(30, content[0], .025, content[2], .035, 0x203040, 0, 'Arial')
            self.robot.setLabel(31, content[1], .025, content[2] + .05, .025, 0x203040, 0, 'Arial')
            self.previous = content


class LabTools:
    """Manage repeatable scenes without adding methods to robot programs."""

    def __init__(self, alvik):
        self.alvik = alvik
        self.robot = alvik.robot
        self.node = self.robot.getSelf()
        self.translation = self.node.getField('translation')
        self.rotation = self.node.getField('rotation')
        self.start = self.translation.getSFVec3f()
        self.heading = self.rotation.getSFRotation()
        orientation = self.node.getOrientation() if hasattr(self.node, 'getOrientation') else (1, 0, 0, 0, 1, 0, 0, 0, 1)
        self.forward = orientation[0], orientation[3]
        box = self.robot.getFromDef('LAB_BOX')
        self.box = box.getField('translation') if box else None
        self.previous_task = self.previous_distance = None
        self.previous_running = False
        self.last_status = None
        self.actual_error = ''

    def reset(self):
        self.alvik.brake()
        self.translation.setSFVec3f(self.start)
        self.rotation.setSFRotation(self.heading)
        self.node.resetPhysics()
        self.alvik._encoder_previous = None
        self.alvik.reset_pose(0, 0, 0)
        self.actual_error = ''

    def update(self):
        state = PROGRAM_NAMESPACE.get('status', {})
        keys = self.alvik._new_keys
        reset_key = bool(keys.intersection((70, 102)))
        start_key = bool(keys.intersection((88, 120)))
        task = state.get('task')
        distance = state.get('box_distance')
        exercise = isinstance(task, int) and task in (0, 1, 2, 3, 4)
        line_exercise = 'gain' in state and 'state' in state
        if exercise and self.box is None:
            node = scene_node(self.robot, 'LAB_BOX', '''
                DEF LAB_BOX Solid { translation 0 0 .065 name "Lab exercise box"
                  children [ Shape { appearance PBRAppearance { baseColor .9 .4 .12 roughness .8 metalness 0 }
                    geometry Box { size .08 .08 .13 } } ]
                  boundingObject Box { size .08 .08 .13 }
                }''')
            self.box = node.getField('translation')
        if self.box and exercise:
            changed = task != self.previous_task or distance != self.previous_distance
            if changed:
                if task == 1:
                    ahead, side = .0794 + (distance if isinstance(distance, (int, float)) else 15) / 100, 0
                elif task == 4:
                    ahead, side = .37, 0
                else:
                    ahead, side = .37, .45
                x = self.start[0] + self.forward[0] * ahead - self.forward[1] * side
                y = self.start[1] + self.forward[1] * ahead + self.forward[0] * side
                self.box.setSFVec3f([x, y, self.start[2] + .064])
            if changed or reset_key or (start_key and task >= 2 and not state.get('running')):
                self.reset()
            if task == 3 and self.previous_running and not state.get('running') and state.get('result', '').startswith('Done'):
                x, y, _ = self.node.getPosition()
                self.actual_error = f' | Webots return error {100 * math.hypot(x - self.start[0], y - self.start[1]):.1f} cm'
                print('[Webots scene]' + self.actual_error, flush=True)
        elif line_exercise and (reset_key or (start_key and state.get('state') == 'WAITING')):
            self.reset()
        elif not exercise and not line_exercise and reset_key:
            self.reset()
            self.node.restartController()
        self.previous_task, self.previous_distance = task, distance
        self.previous_running = state.get('running', False)
        title = state.get('title', 'Lab tools | F restarts from the initial pose') + self.actual_error
        hint = state.get('hint', 'V sensor array | F reset and restart')
        if self.last_status != (title, hint) and self.robot.getFromDef('SCENE_TOOLS') is None:
            self.robot.setLabel(20, title, .025, .025, .035, 0x203040, 0, 'Arial')
            self.robot.setLabel(21, hint, .025, .065, .025, 0x203040, 0, 'Arial')
            self.last_status = title, hint


class HandTarget:
    """Optional movable target, created automatically in worlds without TARGET."""

    def __init__(self, alvik):
        self.alvik, self.robot = alvik, alvik.robot
        node = self.robot.getSelf()
        position, rotation = node.getPosition(), node.getOrientation()
        distance = max(.10, min(.35, .0394 + alvik.get_distance()[2] / 100 - .08))
        x, y = position[0] + rotation[0] * distance, position[1] + rotation[3] * distance
        target = scene_node(self.robot, 'TARGET', f'''
            DEF TARGET Solid {{ translation {x} {y} .06 name "Movable hand target"
              children [ Shape {{ appearance PBRAppearance {{ baseColor .13 .57 .82 roughness .7 metalness 0 }}
                geometry Box {{ size .06 .12 .12 }} }} ]
              boundingObject Box {{ size .06 .12 .12 }}
            }}''')
        self.target = target.getField('translation')

    def update(self):
        keys = self.alvik._new_keys
        offset = .09 * (bool(keys.intersection((75, 107))) - bool(keys.intersection((74, 106))))
        if offset:
            node = self.robot.getSelf()
            position, rotation = node.getPosition(), node.getOrientation()
            x, y, z = self.target.getSFVec3f()
            forward = (x - position[0]) * rotation[0] + (y - position[1]) * rotation[3]
            wanted = max(.10, min(.80, forward + offset))
            x += rotation[0] * (wanted - forward)
            y += rotation[3] * (wanted - forward)
            self.target.setSFVec3f([x, y, z])
        cm = self.alvik.get_distance()[2]
        text = 'no target' if cm >= 349.9 else f'{cm:.1f} cm'
        self.robot.setLabel(40, 'Target: ' + text + ' | J nearer | K farther', .025, .14, .025, 0x203040, 0, 'Arial')


# 3. Scene controllers: presets, keyboard tools, and save-to-run support.
# These helpers run on Webots scene nodes, using this same controller file.


def read_entry(args):
    if not args.getCount():
        return None
    path = (SOURCE_ROOT / args.getMFString(0)).resolve()
    if path.suffix.lower() != '.py' or not path.is_relative_to(SOURCE_ROOT):
        return None
    try:
        return path, path.read_bytes()
    except OSError:
        return None  # Editors may temporarily replace the file during a save.


# Blank Template presets and shared save-to-run support.
# Each box is (centre x/y/z, size x/y/z), in metres.
BOUNDARIES = (
    ((-.85, 1.22, .10), (2.48, .04, .20)),
    ((-.85, -1.22, .10), (2.48, .04, .20)),
    ((-2.07, 0, .10), (.04, 2.40, .20)),
    ((.37, 0, .10), (.04, 2.40, .20)),
)
PRESETS = {
    1: ('Empty', (), (-1.8, 0, .001)),
    2: ('Boundaries', BOUNDARIES, (-1.8, 0, .001)),
    3: ('Centre obstacle', BOUNDARIES + (((-.85, 0, .08), (.18, .18, .16)),), (-1.8, 0, .001)),
    4: ('Maze', BOUNDARIES + (
        ((-.60, -.40, .10), (1.90, .04, .20)),
        ((-1.10, .40, .10), (1.90, .04, .20)),
    ), (-1.8, -.86, .001)),
}
PROP_NAMES = ('PROP_BOX', 'PROP_COLUMN', 'PROP_WALL', 'PROP_BLOCK', 'PROP_BALL', 'PROP_BOUNDARY')


class TemplateScene:
    def __init__(self, world):
        self.world = world
        self.robot = world.getFromDef('ALVIK')
        self.children = world.getFromDef('TEMPLATE_LAYOUT').getField('children')
        self.props = []
        for name in PROP_NAMES:
            node = world.getFromDef(name)
            self.props.append((node, list(node.getField('translation').getSFVec3f()),
                               list(node.getField('rotation').getSFRotation())))

    def apply(self, number):
        title, boxes, start = PRESETS[number]
        while self.children.getCount():
            self.children.removeMF(self.children.getCount() - 1)
        for index, (centre, size) in enumerate(boxes):
            xyz = ' '.join(map(str, centre))
            dimensions = ' '.join(map(str, size))
            self.children.importMFNodeFromString(-1, f'''
                Solid {{ translation {xyz} name "Template fixture {index}"
                  children [ Shape {{
                    appearance PBRAppearance {{ baseColor .38 .44 .52 roughness 1 metalness 0 }}
                    geometry Box {{ size {dimensions} }}
                  }} ]
                  boundingObject Box {{ size {dimensions} }}
                }}''')
        for node, translation, rotation in self.props:
            node.getField('translation').setSFVec3f(translation)
            node.getField('rotation').setSFRotation(rotation)
            node.resetPhysics()
        self.robot.getField('translation').setSFVec3f(list(start))
        self.robot.getField('rotation').setSFRotation([0, 0, 1, 0])
        self.robot.resetPhysics()
        self.world.setLabel(0, 'BLANK TEMPLATE  |  ' + title, .025, .025, .035, 0x203040, 0, 'Arial')
        print(f'[Blank Template] {title}: scene reset; starting the active program.', flush=True)
        self.robot.restartController()


class SourceReload:
    """Wait for saves to settle; syntax errors leave the current run in place."""

    def __init__(self, read_source, robot, title='Blank Template'):
        self.title = title
        self.read_source = read_source
        self.robot = robot
        self.last_source = read_source()
        self.candidate = self.last_source
        self.candidate_since = self.last_poll = 0.

    def update(self, now, requested=False):
        if not requested and now - self.last_poll < .2:
            return
        self.last_poll = now
        source = self.read_source()
        if source != self.candidate:
            self.candidate, self.candidate_since = source, now
        if source is None or (not requested and (source == self.last_source or now - self.candidate_since < .3)):
            return
        path, data = source
        self.last_source = source
        try:
            compile(data, str(path), 'exec')
        except (SyntaxError, ValueError) as error:
            print(f'[{self.title}] Reload skipped: {path.name}: {error}', flush=True)
            return
        print(f'[{self.title}] Restarting {path.name}. Scene kept.', flush=True)
        self.robot.restartController()


def blank_template_tools():
    world = Supervisor()
    scene = TemplateScene(world)
    args = scene.robot.getField('controllerArgs')
    reload = SourceReload(lambda: read_entry(args), scene.robot)
    timestep = int(world.getBasicTimeStep())
    keyboard = world.getKeyboard()
    keyboard.enable(timestep)
    world.setLabel(0, 'BLANK TEMPLATE  |  Empty', .025, .025, .035, 0x203040, 0, 'Arial')
    world.setLabel(1, '1 Empty  |  2 Boundaries  |  3 Centre obstacle  |  4 Maze',
                   .025, .065, .025, 0x203040, 0, 'Arial')
    world.setLabel(2, 'Shift-drag props  |  Save Python or U to run  |  Click the 3D view for keys',
                   .025, .105, .025, 0x203040, 0, 'Arial')
    print('[Blank Template] An idle Alvik on a clear floor; write your own robot behavior in the Python starter.', flush=True)
    print('[Blank Template] Click the 3D view: 1 empty | 2 boundaries | 3 centre obstacle | 4 maze.', flush=True)
    print('[Blank Template] Select and Shift-drag props from the right bay. Presets reset props and restart code.', flush=True)
    source = read_entry(args)
    filename = 'robot_code/' + source[0].relative_to(SOURCE_ROOT).as_posix() if source else 'the active Python file'
    print(f'[Blank Template] Edit {filename} and save to run; U reruns it in the current scene.', flush=True)
    previous_keys = set()
    while world.step(timestep) != -1:
        keys = set()
        key = keyboard.getKey()
        while key != -1:
            keys.add(key & 0xFFFF)
            key = keyboard.getKey()
        pressed = keys - previous_keys
        previous_keys = keys
        chosen = next((number for number in PRESETS if ord(str(number)) in pressed), None)
        if chosen is not None:
            scene.apply(chosen)
        else:
            reload.update(time.monotonic(), bool(pressed & {ord('u'), ord('U')}))


def playground_tools():
    world = Supervisor()
    robot = world.getFromDef('ALVIK')
    args = robot.getField('controllerArgs')
    timestep = int(world.getBasicTimeStep())
    keyboard = world.getKeyboard()
    keyboard.enable(timestep)
    reload = SourceReload(lambda: read_entry(args), robot, title='Playground')
    previous_reload_key = False

    status_field = robot.getBaseNodeField('customData')
    previous_status = None
    world.setLabel(0, 'PLAYGROUND  |  Starting autonomous mode',
                   .025, .025, .035, 0x203040, 0, 'Arial')
    world.setLabel(1, 'X pause / resume  |  C stop; C again changes mode  |  W/A/S/D manual drive',
                   .025, .065, .025, 0x203040, 0, 'Arial')
    world.setLabel(2, 'Drag props from the right-hand bay  |  U or save the active .py file to reload',
                   .025, .105, .025, 0x203040, 0, 'Arial')

    while world.step(timestep) != -1:
        status = status_field.getSFString()
        if status != previous_status:
            world.setLabel(0, 'PLAYGROUND  |  ' + (status or 'Starting autonomous mode'),
                           .025, .025, .035, 0x203040, 0, 'Arial')
            previous_status = status
        reload_key = False
        key = keyboard.getKey()
        while key != -1:
            reload_key |= key & 0xFFFF in (ord('U'), ord('u'))
            key = keyboard.getKey()
        reload_requested = reload_key and not previous_reload_key
        previous_reload_key = reload_key

        reload.update(time.monotonic(), reload_requested)


# 4. Launcher: controller arguments select programs and optional scene tools.


def controller_options(options):
    """Validate separate controllerArgs items and select compatible displays."""
    displays = {'--beams': RangeOverlay, '--hand-target': HandTarget, '--status': ProgramStatus}
    known = {*displays, '--lab'}
    for option in options:
        if option not in known:
            if option.endswith('\\') and option[:-1] in known:
                raise SystemExit(f'Invalid controller argument {option!r}: remove the trailing backslash; use {option[:-1]!r}.')
            raise SystemExit(f'Unknown Alvik controller option: {option!r}. Use separate controllerArgs entries: ' + ', '.join(sorted(known)))
    if len(set(options)) != len(options):
        raise SystemExit('Duplicate Alvik controller argument. Add each flag only once.')
    return (RangeOverlay,) + tuple(displays[option] for option in options if option in displays and option != '--beams'), '--lab' in options


def main():
    global _robot_class, _setup_world, _use_lab_tools
    if sys.argv[1:] == ['--playground-tools']:
        playground_tools()
        return
    if sys.argv[1:] == ['--blank-tools']:
        blank_template_tools()
        return
    if len(sys.argv) < 2:
        raise SystemExit('Set Alvik.controllerArgs to a .py file in robot_code.')
    program = (SOURCE_ROOT / sys.argv[1]).resolve()
    if not program.is_relative_to(SOURCE_ROOT) or program.suffix.lower() != '.py' or not program.is_file():
        raise SystemExit(f'No robot_code script matches {sys.argv[1]!r}. Check controllerArgs.')
    _setup_world, _use_lab_tools = controller_options(sys.argv[2:])
    # Scene tools use supervisor access outside the portable robot scripts.
    _robot_class = Supervisor
    sys.modules['arduino_alvik'] = sys.modules[__name__]
    sys.path.insert(0, str(program.parent))
    if program.parent != SOURCE_ROOT:
        sys.path.insert(1, str(SOURCE_ROOT))
    print(f'[Alvik] Running {program.relative_to(SOURCE_ROOT)}', flush=True)
    previous = {name: getattr(time, name, None) for name in FUNCTIONS}
    for name, function in FUNCTIONS.items():
        setattr(time, name, function)
    previous_argv = sys.argv
    sys.argv = [str(program)]
    try:
        PROGRAM_NAMESPACE.update(__name__='__main__', __file__=str(program), __package__=None)
        exec(compile(program.read_bytes(), str(program), 'exec'), PROGRAM_NAMESPACE)
    except SimulationEnded:
        pass
    finally:
        sys.argv = previous_argv
        for name, function in previous.items():
            if function is None:
                delattr(time, name)
            else:
                setattr(time, name, function)
        if _active is not None and not _active._ended:
            _active.brake()
            _active.robot.step(_active.timestep)

    # A completed user program leaves the robot stopped while V and scene tools
    # remain available. Exceptions still propagate instead of entering this loop.
    if _active is not None and not _active._ended:
        print('[Alvik] Program finished. Robot stopped; V still toggles sensors.', flush=True)
        try:
            while True:
                _active._step()
        except SimulationEnded:
            pass


if __name__ == '__main__':
    main()
