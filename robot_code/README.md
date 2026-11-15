# Alvik programs

Standalone robot behaviors using the supported Arduino Alvik Python API. Open the matching [Webots world](../worlds/README.md) to run a program.

| Program | Purpose |
| --- | --- |
| [black_line.py](black_line.py) | Follow a black track using three line sensors. |
| [blank_template.py](blank_template.py) | Minimal initialized Alvik loop for your own code. |
| [central_routes.py](central_routes.py) | Choose a coloured route and return to Central. |
| [colour_metro.py](colour_metro.py) | Travel to a selected colour destination. |
| [hand_follower.py](hand_follower.py) | Maintain distance from a target using the front sensor. |
| [line_recovery.py](line_recovery.py) | Tune proportional steering and recover a lost line. |
| [maze_escape.py](maze_escape.py) | Scan passages and navigate with right-hand decisions. |
| [obstacle_avoidance.py](obstacle_avoidance.py) | Leave a line, pass an obstacle, and rejoin the track. |
| [playground.py](playground.py) | Switch between manual driving and autonomous exploration. |
| [sensor_motion.py](sensor_motion.py) | Explore LEDs, distance measurements, odometry, and detours. |
| [touch_move.py](touch_move.py) | Drive while directional touch inputs are held. |

The [shared bridge](../controllers/arduino_alvik/README.md) supplies the simulation devices, keyboard input, and visual tools. These programs contain the robot logic. Check [API compatibility](../docs/api-compatibility.md) before transferring a program to physical hardware.
