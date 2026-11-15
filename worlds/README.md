# Webots worlds

Open a `.wbt` file in Webots to run an example. Each world loads the matching Python program from [`robot_code/`](../robot_code/).

| World | What it demonstrates |
| --- | --- |
| [black_line.wbt](black_line.wbt) | Follow a curved black line and stop when the track ends. |
| [blank_template.wbt](blank_template.wbt) | Write your own Alvik code; choose an empty floor, boundaries, a centre obstacle, or a maze. |
| [central_routes.wbt](central_routes.wbt) | Select a colour route at a junction and return to Central. |
| [colour_metro.wbt](colour_metro.wbt) | Select a colour destination and follow the metro track. |
| [hand_follower.wbt](hand_follower.wbt) | Follow a movable target using distance readings; J moves it nearer and K farther. |
| [line_recovery.wbt](line_recovery.wbt) | Compare proportional steering gains and recover the line after a gap. |
| [maze_escape.wbt](maze_escape.wbt) | Check passages, make right-hand decisions, and stop on the green exit. |
| [obstacle_avoidance.wbt](obstacle_avoidance.wbt) | Follow tape, detour around a block, and rejoin the line. |
| [playground.wbt](playground.wbt) | Explore autonomously or drive manually in an arena with movable props. |
| [sensor_motion.wbt](sensor_motion.wbt) | Experiment with LEDs, distance sensing, odometry, and obstacle detours. |
| [touch_move.wbt](touch_move.wbt) | Drive with W/A/S/D and stop forward motion near an obstacle. |

## Supporting files

| File | Purpose |
| --- | --- |
| `<name>.wbt` | The actual scene: robot, objects, floor, devices, and controller arguments. Open this file. |
| `.<name>.wbproj` | Webots editor settings for that scene, including the panel layout and matching Python tab. Keep it beside the `.wbt`; it is not another world or program. |
| `textures/<name>.png` | Painted floor markings used by the matching world. |

For example, `black_line.wbt`, `.black_line.wbproj`, `textures/black_line.png`, and `../robot_code/black_line.py` belong together. The leading dot in a `.wbproj` filename is intentional.

Click the 3D view for keyboard input. **V** shows or hides the sensor array; **X** and **C** represent the Alvik OK and Cancel pads. See the [user guide](../docs/usage.md) for each program's controls and [custom-map instructions](../docs/usage.md#create-a-custom-track).
