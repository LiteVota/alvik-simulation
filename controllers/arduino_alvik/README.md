# Alvik Python bridge

The shared Webots controller connects supported Arduino Alvik API calls to simulated motors and sensors.

| File | Purpose |
| --- | --- |
| [arduino_alvik.py](arduino_alvik.py) | API adapter, simulation timing, keyboard input, sensor displays, scene tools, and program launcher. |
| [runtime.ini](runtime.ini) | Controller runtime configuration; uses the Python command selected in Webots Preferences. |

Keep the robot's `controller` set to `arduino_alvik`. Its first `controllerArgs` entry selects a program under [`robot_code/`](../../robot_code/README.md), such as `black_line.py`.

**V** toggles the sensor array in every world. Optional feature flags and standalone scene-tool modes are listed in the [controller argument reference](../../docs/usage.md#controller-arguments). The [API reference](../../docs/api-compatibility.md) explains supported methods and limitations.
