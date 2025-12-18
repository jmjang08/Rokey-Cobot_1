# Ramen Chef (ROS2 + Flask Recovery UI)
Rokey BootCamp Cobot-1st projects, 2025.11.24~2025.12.05

This project combines a **ROS2 (Humble) robot motion package** with a **Flask + Socket.IO web-based recovery/control UI**.

- **ROS2 package**: `ramen_chef`
  - Executes robot motions via `move_basic.py`
- **Web UI**: `flask_recovery`
  - Button-based control
  - Publishes ROS topics and subscribes to robot state/progress topics

---

## Requirements

### System & ROS
- Ubuntu 22.04
- ROS2 Humble
- `colcon` build tools
- Python 3.8+

> If running on a real robot or simulator, required bringup packages and drivers must be installed separately. \
bringup packages: https://github.com/doosan-robotics/doosan-robot2.git

---

## Python Dependencies (requirements.txt)

All Python dependencies for the web UI are managed via a `requirements.txt` file.

### Install dependencies

```bash
cd ~/cobot1/flask_recovery
python3 -m pip install --user -r requirements.txt
````

> If you are using a virtual environment, activate it before running the command.

---

## ROS2 Build Instructions

```bash
cd ~/cobot1
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

(Optional) Verify that the executable is correctly registered:

```bash
ros2 pkg executables ramen_chef
```

---

## Recommended Execution Order

### Terminal A — Launch ROS2

```bash
cd ~/cobot1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ramen_chef chef.launch.py
```

> If `chef.launch.py` requires arguments (e.g., robot name, host, port, mode, model),
> pass them according to the launch file definition.

---

### Terminal B — Run Flask Web UI

Since the web UI communicates with ROS topics, sourcing the ROS environment is recommended.

```bash
cd ~/cobot1/flask_recovery
source /opt/ros/humble/setup.bash
source ~/cobot1/install/setup.bash

python3 app.py
```

Open your browser:

* Default address: `http://localhost:5000`

---

## Quick Start Summary

### Terminal A

```bash
cd ~/cobot1
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ramen_chef chef.launch.py
```

### Terminal B

```bash
cd ~/cobot1/flask_recovery
source /opt/ros/humble/setup.bash
source ~/cobot1/install/setup.bash
python3 app.py
```

### Browser

* `http://localhost:5000`
