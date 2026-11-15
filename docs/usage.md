# User guide

The example worlds share one Alvik model and one simulation API. The reference setup is Webots R2025a with CPython 3.15.2 on Windows. The project uses the Python standard library and Webots' bundled Python controller library.

## Start here

1. Download and extract the whole repository ZIP so the `worlds/`, `robot_code/` and `controllers/` folders stay together. Keep the `.wbproj` file beside each `.wbt`.
2. Open a `.wbt` in Webots. Its program opens in the Text Editor. If that panel is closed, show it from **View**.
3. Set Webots **Tools > Preferences > General > Startup mode** to **Real-time** if Webots starts paused. The autonomous programs begin as soon as Webots runs and the first sensor frame is ready. If Python fails to start, check **Python command** in the same preferences.
4. Click in the 3D view for keyboard input. **X** is the Alvik **OK** pad. It starts a program that is waiting for input, or pauses and resumes a running program. **C** is the **Cancel** pad. Use **Reload World** for a fresh run.

In every world, **V** toggles the sensor array, grouped distance readings, and guide. These start hidden and require no controller flag. The bridge builds the guide once at startup and switches its visibility. A normally completed program keeps Alvik stopped while the visualization remains usable.

The upper-left of the 3D view always shows a short **V: show/hide sensor array | X: OK | C: Cancel** hint. These X/C labels name the emulated Alvik touch pads; each program supplies their behavior.

| World | Behavior and additional controls |
|---|---|
| `blank_template.wbt` | Idle starter for your own code. **1** empty, **2** boundaries, **3** centre obstacle, **4** maze. Save Python or **U** to run. |
| `maze_escape.wbt` | Starts automatically, checks passages, and stops on the green exit. **X** pauses/resumes, **C** stops, **X** continues. Reload the world for a fresh run. |
| `central_routes.wbt` | Waits at Central. **C** cycles orange, magenta and cyan while docked. **X** starts the round trip, then pauses or resumes it. **C** during a trip pauses it. |
| `black_line.wbt` | Follows the wavy track on startup and stops at the end. |
| `hand_follower.wbt` | Tracks the target on startup. **J** moves the target nearer and **K** moves it farther. |
| `touch_move.wbt` | Hold **W/A/S/D** to drive. Forward stops near the practice box. **X** pauses or enables, **C** stops. |
| `obstacle_avoidance.wbt` | Starts tracking the line on startup, detours around the block and rejoins the tape. **V** toggles the ToF guide. |
| `colour_metro.wbt` | Waits for destination selection. **C** cycles colours while docked. **X** starts or pauses/resumes the trip. **C** during a trip pauses it. |
| `playground.wbt` | Starts autonomously, turning left at obstacles. **C** stops; another **C** while stopped switches manual/autonomous mode. **X** starts or pauses/resumes. In manual mode, hold **W/A/S/D** to drive. **U** or saving the program reloads it. |
| `sensor_motion.wbt` | LED, distance, odometry and obstacle exercises. Use **A/D** to select an exercise. |
| `line_recovery.wbt` | Adjustable line following and recovery. Use **W/S** to choose KP and **X** to start. |

Sources and adaptation details are listed in the [third-party notices](../THIRD_PARTY_NOTICES.md). Target movement, the ToF drawing and code reload are Webots tools outside the robot program.

## Sensor and control experiments

### Sensor Motion

Click the 3D view. **A/D** selects an exercise while the robot is stopped. Selection puts the robot and box in position. **X** runs a motion exercise, pauses, resumes or repeats it. **C** cancels. **F** resets the current exercise.

| Exercise | What to do |
|---|---|
| 1 - LEDs | Tap **W/S**. Each press moves both LEDs one step through red, blue, green, white and violet. |
| 2 - ToF | The box starts 15 cm from the lens. Use **W/S** to choose 8, 15 or 25 cm. The five horizontal readings appear in the console and the ToF HUD. Each side LED uses the nearer of its two readings. It turns green above 15 cm, red at 15 cm or below, and blinks red below 10 cm. A narrow box may fall outside the outer zones. |
| 3a - Straight | Press **X** to drive 20 cm using wheel odometry. |
| 3b - Square | Press **X** to drive a 30 cm square. The console and HUD compare the odometry result with the robot's actual return error in Webots. |
| 4 - Obstacle | Press **X** for the 80 cm course. ToF detects the 8 cm box. The robot detours left and returns to the original path. The preset targets 5-10 cm of side clearance. |

To try a custom obstacle placement, select the exercise first, then move `LAB_BOX` in the Scene Tree before pressing **X**. **F** restores the preset.

### Line Recovery

The HUD shows the selected gain and the current state. While waiting, use **W/S** to choose KP 10, 30, 50 or 80. **X** starts each new run from the same pose and pauses or resumes during the run. **C** stops and returns to `WAITING`. **F** puts the robot back at the start and keeps the selected gain.

The course has a short tape gap. The console records the transitions between `WAITING`, `FOLLOWING` and `SEARCHING` and reports recovery. For a near-centred loss, this simulation exercise first creeps forward across the gap. Otherwise it turns toward the last-seen direction. At the end of the tape the search times out and the robot waits for the next run.

### What Kp changes

Kp is the **proportional steering gain**: how strongly the robot corrects a line error. In `robot_code/line_recovery.py`, the three sensor readings produce a normalized error, and the program applies:

```python
error = (left - right) / (left + middle + right)
correction = Kp * error
left_rpm = 30 - correction
right_rpm = 30 + correction
```

For the same positive error of `0.2`, the requested wheel speeds are:

| Kp | Correction | Left / right wheel speed |
| --- | --- | --- |
| 10 | 2 rpm | 28 / 32 rpm |
| 30 | 6 rpm | 24 / 36 rpm |
| 50 | 10 rpm | 20 / 40 rpm |
| 80 | 16 rpm | 14 / 46 rpm |

The greater speed difference makes a tighter turn. A negative error reverses the correction; zero error commands equal speeds for every Kp. Increasing Kp changes steering strength, while the base speed remains 30 rpm. Motor limits still apply.

Too little gain can give slow corrections and wide turns that lose the tape. Too much can make the robot cross the line repeatedly, zigzag, or lose it after overshooting. The useful value depends on speed, sensor placement, the track, and update timing. Kp reacts to the current error; it does not remember a route or learn a map. The separate line-loss state machine handles the gap.

To compare gains, run the same course at Kp 10, then 30, 50, and 80 using **W/S** while waiting and **X** to start. Use **F** to reset before a comparison. Watch how quickly it corrects at bends and how much it oscillates on straight sections. These values are experiment settings, not a claim that one gain is always best.

## Run your own named Python file

Place a file such as `my_robot.py` under `robot_code/`. In the scene tree, select **ALVIK → controllerArgs**, then change the first item to `my_robot.py`. Keep the robot's `controller` set to `arduino_alvik`. Save the world and reload it. A file can import helpers placed next to it, and several worlds can use the same file. Program paths are relative to `robot_code/` at the repository root, and the name can contain spaces. To point a copied world’s editor tab at a new program, copy the matching hidden `.wbproj` beside it, rename it to match the new world stem and set its `textFiles` entry to the new program.

In a copied `.wbt` world, the same setting looks like this:

    DEF ALVIK Alvik {
      translation 0 0 0.001
      controller "arduino_alvik"
      controllerArgs [ "my_robot.py" ]
    }

Use **Save World As** into the same `worlds/` folder to retain the relative model and texture paths. For a basic custom world, the Python filename is the only argument needed. Optional scene tools are listed below.

A minimal script for a clear arena:

```python
from arduino_alvik import ArduinoAlvik

alvik = ArduinoAlvik()
alvik.begin()
alvik.move(10)
alvik.brake()
```

## Controller arguments

All modes are implemented in `controllers/arduino_alvik/arduino_alvik.py`. Its source has marked sections for the API implementation, Webots displays, scene controllers, and the launcher.

The Alvik node uses a Python filename followed by optional flags, for example `controllerArgs [ "maze_escape.py" "--status" ]`:

| Argument | What it enables | Required world nodes / controls |
| --- | --- | --- |
| `<program.py>` | Runs a program relative to `robot_code/`. | Alvik with the shared controller. |
| `--beams` | Compatibility flag for sensor visualization, now available in all programs without a flag. | **V** toggles it; no special world nodes or imports. |
| `--hand-target` | Movable target and distance labels. | Reuses `TARGET` or creates a target ahead of Alvik; **J** nearer, **K** farther. |
| `--status` | Shows the program's ordinary `status` title and hint on screen. | Without status data, shows the program name and sensor-toggle hint. |
| `--lab` | Exercise labels and repeatable starts for Sensor Motion / Line Recovery. For other programs, **F** resets the initial pose and restarts the program. | Creates `LAB_BOX` when a sensor exercise requests it; placements follow the robot's initial pose. |

All four feature flags can be combined with any robot program, for example `controllerArgs [ "blank_template.py" "--status" "--beams" "--hand-target" "--lab" ]`. Enter each flag as a separate item, without a trailing backslash. Requested objects are created by the bridge instead of requiring a particular world layout. The target and lab tools can change the scene; they do not add navigation behavior to the program. Scene tools use **supervisor TRUE**, now the shared Alvik model's default. Keyboard **X/C**, **W/A/S/D**, and **F** emulate Alvik touch pads; the program determines their behavior, with **F** additionally reserved for lab reset when `--lab` is present.

A separate scene-helper Robot uses the same controller with one standalone tools argument, without a Python filename:

| Argument | What it enables | Required world nodes / controls |
| --- | --- | --- |
| `--playground-tools` | Playground mode labels and source reload. | `ALVIK`; **U** or saving its Python program reruns it. |
| `--blank-tools` | Blank Template labels, scene presets, and source reload. | `ALVIK`, `TEMPLATE_LAYOUT`, `PROP_BOX`, `PROP_COLUMN`, `PROP_WALL`, `PROP_BLOCK`, `PROP_BALL`, `PROP_BOUNDARY`; **1–4** presets, **U** or save to rerun. |

Do not combine a standalone tools argument with a filename or other flags. Robot programs do not need to import or call these scene tools.

## Maze Escape

Open `worlds/maze_escape.wbt`. Alvik starts on the blue square, navigates the physical walls, and brakes on the green exit pad outside the maze. The console and on-screen labels show its current action. **X** pauses or resumes; **C** stops and **X** continues from there. **Reload World** returns to the start for another run.

Press **V** to show or hide the distance readings and array guide. No extra argument is needed. Program text stays at the top-left, with sensor readings stacked below it.

The program checks directions in the order **right → straight → left → back** at each cell centre. Since Alvik's ToF sensor faces forward, it turns to inspect each direction and waits for a fresh reading before deciding. It uses the public gyro readings for measured turns and heading correction, and the public pose estimate for wheel-odometry distance feedback. It does not read Webots position, wall coordinates, or a stored solution.

The supplied maze has 40 cm cell spacing and no loops. `CELL_CM` in `robot_code/maze_escape.py` must match that spacing. `OPEN_CM` separates an adjacent wall from an open passage; `STOP_CM` stops motion if a path becomes blocked. Green is detected through the downward RGB sensor. A move limit and motion timeouts stop the program if it cannot make progress.

This is a cell-maze demonstration. Right-hand following is suitable for the supplied layout, but is not a general solver for every maze with loops or disconnected wall islands. Moving walls or changing the cell spacing can require adjusting the program and start pose. Use the same cell spacing and a reachable green exit pad when making another maze. Sensor thresholds and odometry still require calibration on physical Alvik.

## Blank Template

Open `worlds/blank_template.wbt`. It contains Alvik on a clear square floor with spare props in the tinted bay on the right. `robot_code/blank_template.py` opens in Webots' Text Editor; show that panel from **View** if needed. The starter only initializes Alvik and waits. Add your own Alvik commands in its loop, keeping a short sleep so physics and sensor readings advance.

Save the Python file to run your changes. Click the 3D view and press **U** to rerun the program. Reloading code keeps your scene and robot position. The simulation must be running for saves and keyboard shortcuts to take effect. Syntax errors are reported in the console without replacing the current run. If a program finishes, the scene tools still let you save and run again.

| Key | Scene preset |
| --- | --- |
| **1** | Empty floor. |
| **2** | Square boundary walls. |
| **3** | Boundary walls and one centre obstacle. |
| **4** | A simple maze with alternating openings. |

Choosing a preset resets Alvik to its start, returns the spare props to their bay, replaces the preset walls, and starts the current Python program again. The maze changes the geometry only; your code decides how the robot navigates it. **Reload World** restores the saved world.

Select a spare object, then use its translation handles or hold **Shift** and drag it into the workspace. The bay has a box, cylinder, short wall, low block, rolling ball, and longer boundary segment. Copy/paste a selected Solid to add more. You can also set its Scene Tree `translation` directly. The preset area is 2.4 × 2.4 m, centred at `(-0.85, 0)`; its clear bounds are `x = -2.05…0.35`, `y = -1.2…1.2`. The empty preset has no physical boundary. These are standard [Webots object controls](https://cyberbotics.com/doc/guide/the-user-interface?version=R2025a).

The scene helper lives in the shared controller. Your Python starter uses only the public Alvik API and does not need any preset or editor calls.

The starter contains only imports, initialization, and an idle loop:

```python
from arduino_alvik import ArduinoAlvik
from time import sleep_ms

alvik = ArduinoAlvik()
alvik.begin()

while True:
    # Write your Alvik code here.
    sleep_ms(20)
```

`begin()` initializes the robot. `sleep_ms(20)` yields between iterations; in this bridge it also advances physics and updates sensors. Keep a short sleep in a repeating polling loop instead of using `while True: pass`. The console introduction and scene shortcuts come from the bridge, so they do not clutter the starter.

## Playground

Open `playground.wbt`. Alvik starts in **autonomous mode** inside a walled arena with one orange obstacle in the middle. It drives forward until a front distance reading reaches `STOP_CM` (12 cm), turns left by about 90 degrees using gyro feedback, and continues. It reacts to nearby obstacles and walls; it does not plan a route or guarantee full area coverage. The display shows the selected mode and current action.

Press **X** to pause or resume. Press **C** to stop, then release it and press **C** again to switch between **MANUAL** and **AUTONOMOUS**. Press **X** to start the selected mode. In manual mode, hold **W/A/S/D** to drive and release them to stop. The front sensor blocks forward motion near obstacles while leaving reverse and steering available. Mode changes cancel an unfinished turn; pausing with **X** preserves it.

The tinted bay on the right holds five extra props: a box, a cylinder, a wall, a low block and a ball. Select a prop and drag it into the arena with the translation handles or hold **Shift** while dragging. You can also edit its Scene Tree `translation`. The clear interior extends from about `x = -1.28` to `0.48` and `y = -0.98` to `0.98` metres. The starter obstacle is `CENTER_BOX`; `PROP_BOX` is the spare box in the bay. The ball can roll. The other props stay where you place them. Webots' **Add** action can add further Solids with geometry and a `boundingObject`.

Edit `STOP_CM`, `TURN_ANGLE_DEG`, or `DEFAULT_MODE` in `robot_code/playground.py` and save to compare behaviors. **U** also reloads the active program. The controller restarts in the current scene and returns to `DEFAULT_MODE`; moved props keep their positions. If Webots is paused, resume it to apply the change.

## Create a custom track

Start from `worlds/blank_template.wbt` and edit `worlds/textures/blank_template.png` in Paint or another image editor. Draw lines and colour patches, keep the 1024 × 1024 dimensions, and save as PNG. Use **Reload World** in Webots to load the changed floor. Your program determines how Alvik responds to the markings.

The template's full floor is 4.8 m wide, so a line about 5–6 pixels wide represents roughly 2.5 cm. Use broad bends and a light background for line-following experiments, and place tape under the front line sensors at the robot's start.

To keep the template and create a separate world named `my_course`:

1. In Paint, use **Save As** to save your drawing as `worlds/textures/my_course.png`.
2. In Webots, use **Save World As** to save the template as `worlds/my_course.wbt`. Keep it in the same `worlds/` folder so the relative model paths work.
3. In the new world's Scene Tree, expand **floor → children → Shape → appearance → texture → url** and change `textures/blank_template.png` to `textures/my_course.png`. Save the world.
4. Copy `robot_code/blank_template.py` to `robot_code/my_course.py`. Set **ALVIK → controllerArgs** to `my_course.py`, then save the world and reload it.
5. Open `robot_code/my_course.py` in Webots' Text Editor. For a preset that opens that file automatically, first close the world, then copy the hidden `worlds/.blank_template.wbproj` to `worlds/.my_course.wbproj` and change its `textFiles` entry to `robot_code/my_course.py`. If Webots already created that `.wbproj`, update its entry instead. Reopen the new world. Enable hidden files in your file manager if needed.

The matching files are:

| File | Purpose |
| --- | --- |
| `worlds/my_course.wbt` | The Webots world. |
| `worlds/textures/my_course.png` | Your painted floor. |
| `robot_code/my_course.py` | Your Alvik behavior. |
| `worlds/.my_course.wbproj` | The editor tab setting. |

Edit the new Python file in Webots and save to run it. The copied template retains the scene presets, draggable props, and code reload controls. The common bridge stays shared; no controller copy is needed. Saving a PNG under a new name creates an image; the matching world and its texture URL make it a separate selectable world. Physical obstacles are Webots Solids; painting a shape on the floor does not create a wall. The tinted prop bay is a separate visual overlay, so use the clear area on the left for your drawing.

### Line width and colour routes

Open `worlds/textures/black_line.png` in Paint. Keep the 1024 × 1024 dimensions and save as PNG. In this world, about **9–10 pixels** makes a **2.5–2.7 cm** line. Keep a light background, smooth curves and tape under the front of the robot at its spawn point. Save the PNG and reload the world. To keep multiple courses, save a separate PNG and change the floor's texture URL in a copied world.

In `central_routes.png`, bring each coloured spoke into the grey junction disk so the single floor colour sensor can see it during a spin. Keep the grey shoulders beside the tape, the grey terminal bars and the black approach to Central. The single colour sensor measures the edges of the chosen colour and the robot turns to their midpoint. The three stock line sensors steer by reflectance once the route is selected. Use tape that the line sensors can distinguish from the floor. On return it scans for the black approach. To swap two routes, recolour each whole line including its section inside the disk. The controller has no table of colour headings. The grey disk size and sensor placement determine the centring distance.

## How the bridge works

`controllers/arduino_alvik/arduino_alvik.py` is the single shared Webots controller. It starts the world program, implements the Alvik API adapter, and handles the optional Webots tools. Each world uses one standalone file under `robot_code/`. For example, `set_wheels_speed(30, 30)` translates rpm to Webots motor rad/s, `get_distance()` reports five readings from a single 4 × 4 depth matrix in centimetres, and `sleep_ms(20)` advances simulation time so the next call can read fresh sensors.

Keyboard controls, displays, experiment resets, Blank Template presets, and save-to-run support are defined in that same Python file. Scene-helper Robot nodes run another instance of this controller with a tools argument; they do not require a separate Python module.

The same robot source uses Arduino's MicroPython library on physical Alvik and this adapter under CPython in Webots. The bridge translates calls at runtime; it does not transpile MicroPython source or emulate the firmware. Review the [full compatibility reference](api-compatibility.md) before importing another project. Unsupported methods and MicroPython-only modules are not supplied automatically.

Robot programs do not call Webots devices, ground-truth position, teleport/reset helpers, or display methods. The controller handles the optional experiment scene tools and reads ordinary program status data for display. On hardware, the centre pad resets the program state and odometry estimate; reposition the robot and obstacles manually for repeatable trials. In the supplied experiment worlds, the Webots tools perform that repositioning. A custom program does not need status data or scene tools to use the implemented API.

| Group | Implemented calls |
|---|---|
| Drive | `begin`, `stop`, `brake`, `set_wheels_speed`, `get_wheels_speed`, `get_wheels_position`, `drive`, `get_drive_speed`, `move`, `rotate` |
| Odometry | `get_pose`, `reset_pose` (resets the estimate), wheel speed and position feedback |
| Inputs | `get_line_sensors`, `get_distance`, `get_distance_top`, `get_distance_bottom`, `get_touch_ok`, `get_touch_cancel`, `get_touch_up/down/left/right/center/any` |
| Colour and LEDs | `get_color`, `get_color_label`, `color_calibration`, `rgb2hsv` (hue in degrees), `hsv2label`, `left_led` and `right_led` (also `DL1` and `DL2`; boolean RGB channels) |
| IMU | `get_accelerations` returns `(ax, ay, az)`; `get_gyros` returns `(gx, gy, gz)`; `get_imu` returns all six values in that order. |
| Time module | `time`, `sleep`, `sleep_ms`, `ticks_ms`, `ticks_us`, `ticks_diff`, `ticks_add` |

## Hardware model

The stock robot has one downward RGB sensor, three infrared line sensors, one VL53L7CX front ToF module, one accelerometer/gyroscope IMU, two motor encoders, two RGB LEDs and capacitive touch controls. The model represents these with one RGB sampling device, three ground sensors, one depth device, a gyro and accelerometer, encoders and LEDs. Keyboard input emulates touch-pad presses. The display overlay sits outside the sensor data path.

The VL53L7CX supports 8 × 8 and 4 × 4. Arduino's published Alvik firmware constructs the sensor with the carrier library's default of 4 × 4 at 30 Hz. The firmware sends seven grouped values. Centre is shared between the horizontal and vertical views.

| View | Outputs | Carrier indices, counted from zero |
|---|---|---|
| Horizontal | L, CL, C, CR, R | 4, 5, average of 5 and 6, 6, 7 |
| Vertical | Top, C, Bottom | Average of 0–3, average of 5 and 6, average of 12–14 |

Webots uses one 4 × 4 depth sensor with a nominal 60° × 60° field of view and the same zone grouping. Its refresh period rounds to a whole Webots step. The bridge maps an out-of-range sample to 350 cm. These are rendered depths, so exact distances, material response and timing require comparison with a physical robot.

In every world, **V** shows or hides the guide and readings. The red guide is drawn 18 cm in front of the lens for scale, and the 4 × 4 HUD lists the ungrouped simulated samples in centimetres. The seven grouped results are listed separately. The bridge builds a complete Webots line guide once, without an extra PROTO or special scene nodes, and excludes it from the range finder. Toggling changes visibility; it does not rebuild the mesh. The drawing has no collision geometry and does not change sensor readings.

See the [Arduino Alvik datasheet](https://docs.arduino.cc/resources/datasheets/AKX00066-datasheet.pdf), [ST VL53L7CX datasheet](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf), [Arduino's firmware entry point](https://github.com/arduino-libraries/Arduino_AlvikCarrier/blob/main/examples/firmware/firmware.ino), and [carrier region implementation](https://github.com/arduino-libraries/Arduino_AlvikCarrier/blob/main/src/sensor_tof_matrix.h).

Webots uses rendered depth, colour and rigid-body physics. Exact optical response, the built-in colour illuminator, board/battery telemetry, firmware sensor fusion and hardware timing are outside this model. The bridge implements the listed subset of the API. Physical calibration and hardware-in-the-loop tests are still needed to measure agreement.

## Motor sound

Both wheel motors use Webots' built-in `rotational_motor.wav`, the standard sound used when a robot such as e-puck leaves the motor sound at its default. Webots changes its pitch and volume with motor speed. This is an illustrative motor cue, not a recording of a physical Alvik.

Use Webots' own audio slider or mute control to adjust playback. Robot programs do not need sound calls. The sound is referenced from the installed Webots resources, rather than copied into this project.

### Where the sound is set

Open `protos/Alvik.proto` and find each `RotationalMotor`. Its `sound` field currently contains:

```text
sound "webots://projects/default/worlds/sounds/rotational_motor.wav"
```

This is a file URL field, rather than a menu of sound presets. To try a different sound, change that URL on both wheel motors, save the PROTO, and reload the world. The installed default sounds include:

| Sound | URL |
| --- | --- |
| Rotational motor | `webots://projects/default/worlds/sounds/rotational_motor.wav` |
| Linear motor | `webots://projects/default/worlds/sounds/linear_motor.wav` |
| Your own WAVE loop | For example, `sounds/my_motor.wav`, relative to `protos/Alvik.proto`. Add the file at `protos/sounds/my_motor.wav`. |

The same Webots resource folder also contains bump, roll, and slide audio for contact effects. For a custom motor sample, a short continuous loop avoids a repeated start/stop sound. Webots adjusts the WAVE playback using motor velocity, as described in the [Motor reference](https://cyberbotics.com/doc/reference/motor?version=R2025a).

## README screenshots

The README includes an Alvik close-up with obstacles and a maze view. Use these compositions for future captures:

| Capture | Suggested composition |
| --- | --- |
| Alvik close-up | Pause the robot beside an obstacle, use a low three-quarter angle, and zoom until its body and wheels are easy to see. Leave room in front for the guide. |
| Sensor visualization | Press **V** with an obstacle ahead. Include Alvik, the full red guide, and readable distance values; avoid clipping the robot or guide at the image edge. |
| Maze overview | Use the angled view with the whole maze, blue start, and green exit visible. One capture with **V** on explains the sensor view. |
| Playground / Blank Template | Include the driving area, prop bay, and control hints. Use a shallow overhead angle so the walls and shapes remain distinct. |

Prefer a capture of just the 3D view for the main image. A full workspace image can show Python alongside the simulation, but title bars, editor headers, and console startup lines can contain personal paths. Resolve console errors before taking final captures.

Before adding an image under `docs/images/`, cover every personal path and username with solid, opaque black rectangles. Check the window title, editor title, console commands, and any open dialogs. Cover the full path, including shortened or truncated versions; use black bars rather than blur or translucent marks. Inspect the exported image at full size and remove identifying image metadata. Only the redacted export belongs in the publication project; keep originals outside it.
