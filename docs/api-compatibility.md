# API compatibility

This reference compares the local adapter with the public `ArduinoAlvik` class in [Arduino's MicroPython library](https://github.com/arduino/arduino-alvik-mpy/blob/4674449079545e063d8d98c8fef4eac764b3f7d6/arduino_alvik/arduino_alvik.py), reviewed at commit `4674449079545e063d8d98c8fef4eac764b3f7d6` on 15 November 2026. Arduino library versions can differ; this is the explicit comparison baseline.

## Portability contract

Robot behavior should use public Alvik methods and ordinary Python. The supplied `robot_code/` programs do not import Webots or call scene-reset, ground-truth, or display functions. Webots-specific tools stay in `controllers/`; on hardware, Arduino's own library supplies `ArduinoAlvik`.

Compatibility applies to the implemented methods, options, return shapes, and Python dependencies below. It does not mean that every existing Alvik program works unchanged. Check its imports and API calls first. Create one `ArduinoAlvik()` instance and call `begin()` before using sensors or simulation timing. Add a short `sleep_ms` or `sleep` in polling loops; the adapter advances physics and sensors during these calls rather than running a MicroPython background update thread.

## Important limits

- `move` and `rotate` support blocking operation. `blocking=False` raises `NotImplementedError`; asynchronous target tracking and `is_target_reached` are not implemented. The blocking wrappers return `None`, matching the public library. Keyboard cancellation and motion timeouts stop simulated motors.
- Wheel and drive commands saturate at the simulated motor limits. Supported distance units are `m`, `cm`, `mm`, `inch`, and `in`; angle units are `rad`, `deg`, `rev`, and `revolution`; wheel/angular speed units are `rad/s`, `deg/s`, `rpm`, and `rev/s`. Linear speed uses a distance unit followed by `/s`. Percentage (`%`) wheel-speed and angular-drive commands are not implemented.
- `stop()` brakes the motors, leaving the simulation, LEDs, and input handling active. Arduino's library also turns off the LEDs and stops its background threads. Use `brake()` when the intent is to halt motion and continue the program.
- `get_accelerations()` returns a three-value tuple in g; `get_gyros()` returns a three-value tuple in degrees per second; `get_imu()` returns `(ax, ay, az, gx, gy, gz)`. The fused orientation and gesture APIs are not implemented.
- `get_color('rgb')` returns normalized channels; HSV hue is in degrees, with saturation/value in 0..1. `hsv2label` follows the reviewed Arduino classifier, including labels such as `LIGHT BLUE` and `LIGHT GREY`. The route programs define their own tape classifiers from public colour readings to distinguish their route names.
- `color_calibration` stores rendered black/white reference samples. Its sampling, illumination, and normalization are approximations of the hardware procedure. Raw hardware colour counts and illuminator control are not implemented.
- `left_led`, `right_led`, `DL1`, and `DL2` provide `set_color(red, green, blue)`. Channels are on/off, as in Arduino's library; fractional values do not provide hardware PWM brightness.
- Touch methods return booleans. The keyboard maps to touch pads; the bridge retains brief OK/Cancel taps until polled so they are not lost between simulation steps.
- `reset_pose` changes the odometry estimate, not the robot's physical position. Repeatable teleporting in experiment worlds belongs to the Webots scene tools and is not an extra Alvik method.
- Python's `time.time`, `sleep`, `sleep_ms`, `ticks_ms`, `ticks_us`, `ticks_diff`, and `ticks_add` use simulation time while a robot program runs. `time.time()` reports elapsed simulation seconds, not a real date. Durations round to whole world steps. Unsupported timing functions and busy loops can have different behavior.
- `machine`, `network`, `espnow`, firmware UART/I2C access, and external-device libraries are not emulated. The physical Alvik library must not be copied over the local adapter. Pure-Python helpers may be placed beside a robot script.
- Servos, battery/charging state, firmware/version management, built-in LED/illuminator control, event callbacks and timers, and the individual `left_wheel`/`right_wheel`, `servo_A`/`servo_B`, and `i2c` objects are not implemented. Missing APIs are not replaced with invented readings or successful no-ops.

## Public method coverage

The following implemented calls are available but are not called by the supplied robot examples: `color_calibration`, `get_accelerations`, `get_color_label`, `get_distance_bottom`, `get_distance_top`, `get_drive_speed`, `get_imu`, `get_touch_any`, `get_wheels_speed`, `hsv2label`, `move`, and `rotate`. The distance overlay may use internal sensor reads; that does not make these public calls part of a robot example.

“Used” means a method is called by a supplied robot program, including its button polling. “Not used” does not mean unsupported. “Limited” refers to the option/behavior restrictions above. Supporting a method name does not certify calibration, timing, or physical equivalence.

| Public method | Adapter | Used by examples |
| --- | --- | --- |
| `begin` | Implemented | Yes |
| `brake` | Implemented | Yes |
| `check_firmware_compatibility` | Not implemented | No |
| `color_calibration` | Limited | No |
| `drive` | Limited | Yes |
| `get_accelerations` | Implemented | No |
| `get_ack` | Not implemented | No |
| `get_battery_charge` | Not implemented | No |
| `get_color` | Implemented | Yes |
| `get_color_label` | Implemented | No |
| `get_color_raw` | Not implemented | No |
| `get_distance` | Implemented | Yes |
| `get_distance_bottom` | Implemented | No |
| `get_distance_top` | Implemented | No |
| `get_drive_speed` | Limited | No |
| `get_fw_version` | Not implemented | No |
| `get_gyros` | Implemented | Yes |
| `get_imu` | Implemented | No |
| `get_lib_version` | Not implemented | No |
| `get_lifted` | Not implemented | No |
| `get_line_sensors` | Implemented | Yes |
| `get_orientation` | Not implemented | No |
| `get_pose` | Implemented | Yes |
| `get_required_fw_version` | Not implemented | No |
| `get_servo_positions` | Not implemented | No |
| `get_shake` | Not implemented | No |
| `get_tilt` | Not implemented | No |
| `get_touch_any` | Implemented | No |
| `get_touch_cancel` | Implemented | Yes |
| `get_touch_center` | Implemented | Yes |
| `get_touch_down` | Implemented | Yes |
| `get_touch_left` | Implemented | Yes |
| `get_touch_ok` | Implemented | Yes |
| `get_touch_right` | Implemented | Yes |
| `get_touch_up` | Implemented | Yes |
| `get_version` | Not implemented | No |
| `get_wheels_position` | Implemented | Yes |
| `get_wheels_speed` | Implemented | No |
| `hsv2label` | Implemented | No |
| `is_battery_charging` | Not implemented | No |
| `is_on` | Not implemented | No |
| `is_target_reached` | Not implemented | No |
| `move` | Limited | No |
| `on_drop` | Not implemented | No |
| `on_lift` | Not implemented | No |
| `on_nx_tilt` | Not implemented | No |
| `on_ny_tilt` | Not implemented | No |
| `on_nz_tilt` | Not implemented | No |
| `on_shake` | Not implemented | No |
| `on_touch_cancel_pressed` | Not implemented | No |
| `on_touch_center_pressed` | Not implemented | No |
| `on_touch_down_pressed` | Not implemented | No |
| `on_touch_left_pressed` | Not implemented | No |
| `on_touch_ok_pressed` | Not implemented | No |
| `on_touch_right_pressed` | Not implemented | No |
| `on_touch_up_pressed` | Not implemented | No |
| `on_x_tilt` | Not implemented | No |
| `on_y_tilt` | Not implemented | No |
| `on_z_tilt` | Not implemented | No |
| `print_status` | Not implemented | No |
| `reset_pose` | Implemented | Yes |
| `rgb2hsv` | Implemented | Yes |
| `rotate` | Limited | No |
| `send_ack` | Not implemented | No |
| `set_behaviour` | Not implemented | No |
| `set_builtin_led` | Not implemented | No |
| `set_illuminator` | Not implemented | No |
| `set_servo_positions` | Not implemented | No |
| `set_timer` | Not implemented | No |
| `set_wheels_position` | Not implemented | No |
| `set_wheels_speed` | Limited | Yes |
| `stop` | Limited | Yes |
| `timer` | Not implemented | No |

## Verification status

The table records source-level coverage against the pinned Arduino library and calls made by the supplied examples. It does not establish motion accuracy, sensor calibration, or timing equivalence. Validate those properties in Webots and on a physical Alvik for the intended task.
