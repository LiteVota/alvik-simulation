# User guide

Run the examples with Webots R2025a and Python 3. The reference setup uses CPython 3.15.2 on Windows. The project uses Python's standard library and Webots' bundled controller library; no separate Alvik package is needed.

## Start here

1. Download and extract the repository ZIP. Keep its folder structure, including the hidden `.<name>.wbproj` files beside the worlds.
2. Open `worlds/black_line.wbt` in Webots. The matching Python program opens in the Text Editor; enable **View > Text Editor** if the panel is closed.
3. Start or resume the simulation using Webots' run controls. Black Line begins after its sensors are ready.
4. Click the 3D view for keyboard input. In Black Line, **X** pauses or resumes and **C** stops the robot. Use **Reload World** to restore the saved scene and restart the program.

If Python does not start, set **Tools > Preferences > General > Python command** to your Python executable. Keep the shared controller in place; install Arduino's library only on the physical robot.

## World controls

**X/C**, **W/A/S/D**, and **F** represent Alvik's OK, Cancel, directional, and centre touch pads. Each program decides how to use them. The simulation must be running and the 3D view focused for keyboard controls to work.

| World | Startup and controls |
| --- | --- |
| `black_line.wbt` | Follows the tape automatically and stops when it loses the track. **X** pauses/resumes or re-enables motion after a stop; **C** stops. |
| `blank_template.wbt` | Idle starter. **1–4** selects scene presets; saving Python or **U** reruns the program. |
| `central_routes.wbt` | Waits at Central. **C** cycles orange, magenta, and cyan while docked; **X** starts a round trip or pauses/resumes it. **C** during a trip pauses it. |
| `colour_metro.wbt` | Waits at a terminal. **C** cycles cyan, magenta, and yellow destinations while docked; **X** starts a trip or pauses/resumes it. **C** during a trip pauses it. |
| `hand_follower.wbt` | Tracks a target automatically to maintain a 10 cm gap. **J/K** moves the target nearer/farther. **X** pauses/resumes; **C** stops. |
| `line_recovery.wbt` | Waits for input. **W/S** selects a steering gain; **X** starts or pauses/resumes; **C** cancels; **F** resets. |
| `maze_escape.wbt` | Starts automatically and stops on the green exit. **X** pauses/resumes; **C** stops and **X** continues. Reload for a fresh run after completion or failure. |
| `obstacle_avoidance.wbt` | Follows the line automatically, detours around the block, and rejoins the tape. **X** pauses/resumes; **C** stops. |
| `playground.wbt` | Starts autonomously. **C** stops; another **C** while stopped changes manual/autonomous mode. **X** starts or pauses/resumes. In manual mode, hold **W/A/S/D** to drive. **U** or saving Python reloads the program. |
| `sensor_motion.wbt` | Starts with the LED exercise. **A/D** selects an exercise; **W/S** changes LED colour or target distance; **X** runs or pauses/resumes motion; **C** cancels; **F** resets. |
| `touch_move.wbt` | Hold **W/A/S/D** to drive; release to stop. Forward motion is blocked near an obstacle. **X** pauses/enables; **C** stops. |

The [world catalog](../worlds/README.md) describes the scenes and supporting files. The following sections explain the configurable examples and editing tools.

### Sensor visualization

In any world, **V** shows or hides the red sensor guide, the 4 × 4 distance array, and the grouped readings. They start hidden. The values are in centimetres; `MAX` means the sample is at or beyond the simulated 350 cm range.

The red grid is drawn 18 cm in front of the lens as a scale guide. It does not show the maximum sensing distance or form a physical obstacle. The guide is excluded from the depth sensor's view. A normally completed program leaves the robot stopped while visualization remains available.

## Sensor and control experiments

### Sensor Motion

Open `worlds/sensor_motion.wbt`. Use **A/D** to select an exercise. Cancel a running motion exercise with **C** before selecting another. The scene tools position Alvik and the practice box for the selected exercise. **X** starts a motion, pauses/resumes it, or repeats it after completion; **F** returns the robot to its start.

| Exercise | What to do |
| --- | --- |
| 1 — LEDs | Tap **W/S** to cycle both LEDs through red, blue, green, white, and violet. |
| 2 — ToF | The box starts 15 cm from the lens. Use **W/S** to select 8, 15, or 25 cm. The console prints the five horizontal readings; **V** shows them on screen. Each side LED uses the nearer of its two readings: green above 15 cm, red at 15 cm or below, and blinking red below 10 cm. |
| 3a — Straight | Press **X** to drive 20 cm using wheel odometry. |
| 3b — Square | Press **X** to drive a 30 cm square. The console reports odometry; the Webots display also reports the actual return error. |
| 4 — Obstacle | Press **X** for the 80 cm course. Alvik detects the 8 cm box, detours left, and returns to the original path. |

To try another obstacle placement, select the exercise and then move `LAB_BOX` in the Scene Tree before pressing **X**. **F** resets the robot; switch to another exercise and back to restore the box preset. A narrow box may fall outside the outer ToF zones.

### Line Recovery

Open `worlds/line_recovery.wbt`. While waiting, use **W/S** to choose Kp 10, 30, 50, or 80. **X** starts from the same pose and pauses/resumes during a run. **C** stops and returns to `WAITING`; **F** resets the robot while keeping the selected gain.

The console records `WAITING`, `FOLLOWING`, and `SEARCHING` states. When the line disappears near the centre, the robot first creeps forward to cross a short gap. Otherwise it searches toward the last-seen direction. If it cannot reacquire the tape, it stops and waits for another run.

### What Kp changes

Kp is the proportional steering gain: how strongly the robot corrects a line error. When tape is detected, `robot_code/line_recovery.py` applies:

```python
error = (left - right) / (left + middle + right)
correction = Kp * error
left_rpm = 30 - correction
right_rpm = 30 + correction
```

For a positive error of `0.2`, the requested wheel speeds are:

| Kp | Correction | Left / right wheel speed |
| --- | --- | --- |
| 10 | 2 rpm | 28 / 32 rpm |
| 30 | 6 rpm | 24 / 36 rpm |
| 50 | 10 rpm | 20 / 40 rpm |
| 80 | 16 rpm | 14 / 46 rpm |

A larger speed difference makes a tighter turn. A negative error reverses the correction; zero error gives equal wheel speeds. The base speed remains 30 rpm.

Compare the gains on the same course, using **F** before each run. Low gain may correct too slowly at bends; high gain may cause oscillation or overshoot. The useful gain depends on speed, track shape, sensor placement, and update timing. The separate line-loss state machine handles the gap.

## Run your own named Python file

1. Add a file such as `my_robot.py` under `robot_code/`.
2. In Webots' Scene Tree, select **ALVIK → controllerArgs** and change the first entry to `my_robot.py`. Keep `controller` set to `arduino_alvik`.
3. Save and reload the world, then open your Python file in the Text Editor.

In a `.wbt` file, the setting looks like this:

```text
DEF ALVIK Alvik {
  translation 0 0 0.001
  controller "arduino_alvik"
  controllerArgs [ "my_robot.py" ]
}
```

Program paths are relative to `robot_code/`. Helpers can live beside the program, and several worlds can select the same file. Keep copied worlds in `worlds/` so their relative model and texture paths work. See [Create a custom track](#create-a-custom-track) for the matching editor preset.

A minimal program for a clear arena:

```python
from arduino_alvik import ArduinoAlvik

alvik = ArduinoAlvik()
alvik.begin()
alvik.move(10)  # Centimetres; blocking.
alvik.brake()
```

Create one `ArduinoAlvik` instance and call `begin()` before using sensors or timing helpers. Keep a short `sleep_ms` or `sleep` in polling loops so the bridge can advance physics, update input, and refresh sensors. A busy loop prevents those updates.

Saving Python automatically reruns it only in Playground and Blank Template, where scene helpers watch the selected file. In other supplied worlds, save the file and reload the world to run it. Custom code must poll the touch methods to implement its own start/pause controls.

Before transferring code to physical Alvik, review [API compatibility](api-compatibility.md). Use Arduino's library on hardware and check sensor thresholds, calibration, and motion settings on the robot.

## Controller arguments

The shared controller is `controllers/arduino_alvik/arduino_alvik.py`. Its first argument selects the robot program; optional flags follow as separate entries:

```text
controllerArgs [ "maze_escape.py" "--status" ]
```

| Argument | Effect |
| --- | --- |
| `<program.py>` | Selects a program relative to `robot_code/`. |
| `--status` | Displays the program's `status` title and hint; falls back to its name when no status data is supplied. |
| `--hand-target` | Reuses `TARGET` or creates a target ahead of Alvik. **J/K** moves it nearer/farther. |
| `--lab` | Provides repeatable starts and labels for Sensor Motion and Line Recovery, creating `LAB_BOX` if needed. For other programs, **F** resets the initial pose and restarts the program. |
| `--beams` | Retained for compatibility. Sensor visualization is available with **V** without this flag. |

The four flags can be combined, each at most once. The shared model uses `supervisor TRUE` for scene tools. Target and lab tools may reposition objects; navigation remains the program's responsibility. With `--lab`, **F** is also reserved for scene reset.

The editable worlds also contain a separate supervisor Robot, `SCENE_TOOLS`, which runs the same controller with one standalone argument:

| Argument | Required nodes and controls |
| --- | --- |
| `--playground-tools` | Requires `ALVIK`. Displays Playground status; **U** or saving its active Python file reruns it. |
| `--blank-tools` | Requires `ALVIK`, `TEMPLATE_LAYOUT`, `PROP_BOX`, `PROP_COLUMN`, `PROP_WALL`, `PROP_BLOCK`, `PROP_BALL`, and `PROP_BOUNDARY`. Adds **1–4** presets and **U**/save reload. |

Keep these standalone arguments on the scene-helper Robot, without a program filename or other flags. Robot programs do not need to call the scene tools.

## Maze Escape

Open `worlds/maze_escape.wbt`. Alvik starts on the blue square and navigates the physical walls until its downward RGB sensor detects the green exit pad. **X** pauses/resumes; **C** stops and **X** continues. After completion or failure, reload the world for another run.

At each cell centre, the program checks **right → straight → left → back**. It turns to face each candidate direction and waits for fresh ToF readings. Gyro feedback measures turns; wheel odometry measures cell travel. The program has no stored map or exit coordinates.

The supplied maze uses 40 cm cells. If you change the layout, keep `CELL_CM` in `robot_code/maze_escape.py` consistent with the spacing and place a reachable green exit pad. `OPEN_CM` distinguishes a wall from a passage; `STOP_CM` protects against blocked motion. Move limits and timeouts stop the robot when it cannot make progress.

Right-hand following suits the supplied maze. It does not guarantee an exit for every layout with loops or disconnected wall islands. Changing walls, spacing, or the start pose may require retuning thresholds.

## Blank Template

Open `worlds/blank_template.wbt`. Alvik waits on a clear floor, with spare props in the tinted bay on the right. The starter, `robot_code/blank_template.py`, initializes Alvik and then idles:

```python
from arduino_alvik import ArduinoAlvik
from time import sleep_ms

alvik = ArduinoAlvik()
alvik.begin()

while True:
    # Write your Alvik code here.
    sleep_ms(20)
```

Save your Python file to run it, or click the 3D view and press **U** to rerun. Reloading code keeps the robot and props in place. The simulation must be running. Syntax errors are reported in the console and leave the current run in place.

| Key | Scene preset |
| --- | --- |
| **1** | Empty floor. |
| **2** | Square boundary walls. |
| **3** | Boundary walls and one centre obstacle. |
| **4** | A maze with alternating openings. |

Choosing a preset resets Alvik, returns spare props to their bay, replaces the preset walls, and restarts the selected Python program. Your code determines how the robot navigates the new geometry. **Reload World** restores the saved scene.

Select a spare prop and use its translation handles or **Shift-drag** to move it into the arena. The bay contains a box, cylinder, short wall, low block, rolling ball, and longer boundary segment. Copy/paste a selected Solid to add more, or edit its Scene Tree `translation`.

The preset area is 2.4 × 2.4 m, centred at `(-0.85, 0)`: its clear bounds are `x = -2.05…0.35` and `y = -1.2…1.2` metres. The empty preset has no physical boundary. See [Webots object controls](https://cyberbotics.com/doc/guide/the-user-interface?version=R2025a) for editor operations.

## Playground

Open `worlds/playground.wbt`. Alvik starts in autonomous mode inside a walled arena with an orange obstacle. When a front reading reaches `STOP_CM` (12 cm), it turns left about 90° using gyro feedback and continues. This is reactive exploration; it does not plan a route or guarantee full area coverage.

**X** pauses/resumes. **C** stops; release it and press **C** again to switch manual/autonomous mode. Press **X** to start the selected mode. In manual mode, hold **W/A/S/D** to drive and release them to stop. The front sensor blocks forward motion near obstacles while reverse and steering remain available. Changing mode cancels an unfinished turn; pausing preserves it.

Move spare props from the right-hand bay using translation handles or **Shift-drag**. The clear interior extends approximately from `x = -1.28` to `0.48` and `y = -0.98` to `0.98` metres. `CENTER_BOX` is the starting obstacle; `PROP_BOX` is the spare box. The ball can roll; other props remain where placed.

Edit `STOP_CM`, `TURN_ANGLE_DEG`, or `DEFAULT_MODE` in `robot_code/playground.py` and save. Saving or **U** restarts the program in `DEFAULT_MODE`, keeping the current robot and prop positions.

## Create a custom track

Edit `worlds/textures/blank_template.png` in Paint or another image editor. Keep its 1024 × 1024 dimensions and save as PNG. Reload the world to see the floor changes. Use the clear area on the left; the tinted prop bay is a separate overlay.

The full template floor is 4.8 m wide, so a 5–6 pixel line represents roughly 2.3–2.8 cm. Use a light background, broad bends, and tape under the front line sensors at the start. Floor markings do not create physical walls; use 3D props for obstacles.

To preserve the template and create `my_course`:

1. Save the drawing as `worlds/textures/my_course.png`.
2. In Webots, use **Save World As** to save the template as `worlds/my_course.wbt`.
3. In the new world's Scene Tree, expand **floor → children → Shape → appearance → texture → url**. Change `textures/blank_template.png` to `textures/my_course.png`.
4. Copy `robot_code/blank_template.py` to `robot_code/my_course.py`. Set **ALVIK → controllerArgs** to `my_course.py`, save the world, and reload.
5. Open the new Python file in the Text Editor. To make it open automatically, close the world, then update `worlds/.my_course.wbproj`. Copy `worlds/.blank_template.wbproj` under that name if needed and set `textFiles: 0 "robot_code/my_course.py"`. Reopen the world.

Enable hidden files in your file manager to find the `.wbproj`. Keep these four files together:

| File | Purpose |
| --- | --- |
| `worlds/my_course.wbt` | Scene and controller selection. |
| `worlds/textures/my_course.png` | Painted floor. |
| `robot_code/my_course.py` | Robot behavior. |
| `worlds/.my_course.wbproj` | Editor settings and Python tab. |

The copied template retains scene presets, draggable props, and save-to-run support. Edit the new program and save to run it.

### Line width and colour routes

In `worlds/textures/black_line.png`, the 2.8 m floor gives a different scale: a 9–10 pixel line is about 2.5–2.7 cm. Keep the image at 1024 × 1024 pixels and place tape under the front sensors at the start.

In `central_routes.png`, each coloured spoke must extend into the grey junction disk so the single colour sensor can detect it during a scan. Keep the grey shoulders, terminal bars, and black approach to Central. The program finds the edges of the selected colour, turns toward their midpoint, and then follows tape with the three line sensors. To exchange routes, recolour each whole spoke, including its portion inside the disk.

## How the bridge works

`controllers/arduino_alvik/arduino_alvik.py` supplies the Alvik API, simulation timing, keyboard input, displays, scene tools, and program launcher. Each world selects a standalone behavior under `robot_code/`. For example, `set_wheels_speed(30, 30)` converts rpm to Webots motor rad/s; `get_distance()` returns five horizontal distances in centimetres.

The bridge runs compatible source under CPython. On physical Alvik, the same import loads Arduino's MicroPython library. The [API reference](api-compatibility.md) defines supported calls, units, return values, and dependency limits.

The supplied robot programs use public Alvik methods. Scene placement, ground-truth measurements, visual overlays, and physical resets stay in the Webots controller. On hardware, reset actions in the experiment programs reset their state and odometry estimate; reposition the robot and obstacles manually for repeatable trials.

## Hardware model

The model represents differential drive, two encoders, three infrared line sensors, one downward RGB sensor, one front depth sensor, an accelerometer/gyroscope, two RGB LEDs, and keyboard touch input.

The front device renders a 4 × 4 depth matrix over a nominal 60° × 60° field of view. Its grouping follows the Arduino carrier's 4 × 4 configuration:

| View | Outputs | Matrix indices, counted from zero |
| --- | --- | --- |
| Horizontal | L, CL, C, CR, R | 4; 5; average of 5 and 6; 6; 7. |
| Vertical | Top, C, Bottom | Average of 0–3; average of 5 and 6; average of 12–14. |

The centre is shared, giving seven distinct grouped readings. The nominal 30 Hz refresh rounds up to a whole Webots step. Samples beyond the 3.5 m range are capped at 350 cm by the adapter.

See the [Arduino Alvik datasheet](https://docs.arduino.cc/resources/datasheets/AKX00066-datasheet.pdf), [ST VL53L7CX datasheet](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf), [Arduino carrier firmware](https://github.com/arduino-libraries/Arduino_AlvikCarrier/blob/main/examples/firmware/firmware.ino), and [region implementation](https://github.com/arduino-libraries/Arduino_AlvikCarrier/blob/main/src/sensor_tof_matrix.h) for hardware references.

Geometry, friction, optical response, and timing include approximations. The model has not been calibrated against a physical Alvik. Firmware sensor fusion, the colour illuminator, battery telemetry, and external hardware are outside the implemented API subset.

## Motor sound

Both wheel motors reference Webots' built-in `rotational_motor.wav`. Pitch and volume follow motor speed; adjust playback with Webots' audio control. The sound is an illustrative cue and is supplied by the installed Webots distribution.

To change it, edit the `sound` field on both `RotationalMotor` nodes in `protos/Alvik.proto`, then save and reload the world:

```text
sound "webots://projects/default/worlds/sounds/rotational_motor.wav"
```

For a custom WAVE loop at `protos/sounds/my_motor.wav`, use `sound "sounds/my_motor.wav"`. See the [Webots Motor reference](https://cyberbotics.com/doc/reference/motor?version=R2025a) for playback behavior.
