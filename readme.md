# Capra SpaceMouse Package

This ROS 2 package provides a node that publishes input from a SpaceMouse as a `sensor_msgs/msg/Joy` message. It handles live updates from the device and will recover even if the device is temporarily disconnected.

## Overview

The main node is `spacemouse_joy.py`. It:
- Connects to a 3Dconnexion SpaceMouse using `pyspacemouse`.
- Reads 6-DOF motion and 2-button states.
- Publishes them at 100 Hz as `Joy` messages on the `joy` topic.
- Handles disconnections and attempts to reconnect every second, publishing zeros until the device is restored.

## Node: `spacemouse_joy.py`

### Purpose

Publishes 3D SpaceMouse data to the `/spacemouse_joy` topic as a standard `sensor_msgs/msg/Joy` message.

### Published

- **Topic**: `/spacemouse_joy`  
  **Type**: `sensor_msgs/msg/Joy`  
  **Frequency**: ~100 Hz  
  **Content**:
  - `axes[0]` — Translation Y (forward/backward)
  - `axes[1]` — Translation X (left/right)
  - `axes[2]` — Translation Z (up/down)
  - `axes[3]` — Rotation Roll (left/right)
  - `axes[4]` — Rotation Pitch (forward/backward)
  - `axes[5]` — Rotation Yaw (twist)
  - `buttons[0]` — Button 1 (left)
  - `buttons[1]` — Button 2 (right)
  
  The axes order have been adjusted to fit the order expected from twist.

| Index | Field         | Type    | Description                    |
|-------|---------------|---------|--------------------------------|
| 0     | axes[0]       | `float` | Translate forward/backward     |
| 1     | axes[1]       | `float` | Translate left/right           |
| 2     | axes[2]       | `float` | Translate up/down              |
| 3     | axes[3]       | `float` | Rotate roll (tilt left/right)  |
| 4     | axes[4]       | `float` | Rotate pitch (tilt fore/aft)   |
| 5     | axes[5]       | `float` | Rotate yaw (twist)             |
| 0     | buttons[0]    | `int`   | Button 1 (left)                |
| 1     | buttons[1]    | `int`   | Button 2 (right)               |

### Behavior

- On startup: attempts to connect to the SpaceMouse.
- On each timer tick (10 ms): reads and publishes motion/button data.
- On error: publishes zeros and starts a 1 Hz reconnect timer.
- On successful reconnection: resumes normal publishing.

## Launch

All nodes can be launched using:

```bash
ros2 run your_package_name spacemouse_joy.py