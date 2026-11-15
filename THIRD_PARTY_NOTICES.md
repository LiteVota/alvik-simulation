# Third-party notices

This project includes adapted Arduino examples and CAD-derived model assets. Their sources and license terms are listed below. No endorsement by the original authors is implied.

## Arduino Alvik MicroPython examples

- Source: [arduino/arduino-alvik-mpy](https://github.com/arduino/arduino-alvik-mpy)
- Reference reviewed: [`4674449079545e063d8d98c8fef4eac764b3f7d6`](https://github.com/arduino/arduino-alvik-mpy/tree/4674449079545e063d8d98c8fef4eac764b3f7d6)
- License: Mozilla Public License 2.0; full text in [LICENSE](LICENSE).

`robot_code/black_line.py`, `hand_follower.py`, and `touch_move.py` adapt the corresponding factory demos under `examples/demo/`. The centroid calculation and proportional steering in `obstacle_avoidance.py` also derive from the Arduino line-follower example. `line_recovery.py` uses the equivalent normalized three-sensor error with a selectable gain and a state machine.

The local `black_line.py` corresponds to upstream `examples/demo/line_follower.py`.

Changes include Webots sensor and motor access through the local adapter, consistent OK/Cancel controls, manual hold-to-drive behavior, bounded target following, line-loss detection, obstacle detours, and simulation-specific reset and display tools. The local adapter implements the documented API subset; it does not bundle the physical robot's firmware or MicroPython runtime. Modified source is included in this repository.

The adapter's `rgb2hsv` and `hsv2label` implementations also derive from Arduino's library. They retain its degree-based hue scale and public colour-label behavior.

## Arduino Alvik reference design

- Creator: Arduino
- Source: [official Alvik resource page](https://docs.arduino.cc/hardware/alvik/)
- Archive: [AKX00066-step.zip in Arduino's documentation repository](https://github.com/arduino/docs-content/blob/main/content/hardware/edu/solution-and-kits/alvik/downloads/AKX00066-step.zip)
- Source file: `alvik_reference_design.step`
- License source: [Arduino documentation LICENSE.md](https://github.com/arduino/docs-content/blob/main/LICENSE.md), whose full license text is Creative Commons Attribution-ShareAlike 4.0 International.
- License copy: [LICENSES/Arduino-CC-BY-SA-4.0.md](LICENSES/Arduino-CC-BY-SA-4.0.md).

The four meshes in `protos/meshes/alvik/` are derived visual geometry. Changes include tessellation into OBJ format, separation into body, PCB, and wheel meshes, and alignment within the Webots model. The assembled `protos/Alvik.proto` adds Webots joints, devices, collision geometry, and approximate physical parameters. These model assets are distributed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/). Arduino's trademarks remain the property of their owners.

## Runtime dependencies and technical references

Wheel motors reference Webots' built-in rotational-motor sound through a `webots://` URL. That audio remains part of the separately installed Webots distribution and is not bundled or relicensed here.

Webots and Python are installed separately and retain their own licenses. Worlds reference Webots' built-in background and lighting PROTOs through `webots://` URLs. Arduino's Alvik documentation, carrier firmware, and ST's VL53L7CX documentation inform the device model; links are in the [user guide](docs/usage.md#hardware-model).
