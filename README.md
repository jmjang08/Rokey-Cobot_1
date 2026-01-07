# 🍜 Ramen Chef

> Rokey Bootcamp Cobot-1st Project | 🗓️: 2025.11.24 ~ 2025.12.05

This project combines a **ROS2 (Humble) robot motion package** with a **Flask + Socket.IO web-based recovery/control UI** to implement a ramen-cooking robot system.

* 🤖 **ROS2 package**: `ramen_chef`
* Executes basic robot motions via `move_basic.py`.


* 🌐 **Web UI**: `flask_recovery`
* Provides an intuitive button-based control interface.
* Publishes ROS topics to send commands and subscribes to robot state/progress topics for real-time display.

---

## ⚙️ Requirements

### 💻 System & ROS

* Ubuntu 22.04
* ROS2 Humble
* `colcon` build tools
* Python 3.8+

> ⚠️ **Note**: To run this on a real robot or simulator, the corresponding bringup packages and drivers must be installed separately.
> * Bringup packages: [doosan-robot2 GitHub](https://github.com/doosan-robotics/doosan-robot2.git)
> 
> 

---

## 🐍 Python Dependencies

All Python dependencies for the web UI are managed via a `requirements.txt` file.

### Install dependencies

```bash
cd ~/cobot1/flask_recovery
python3 -m pip install --user -r requirements.txt
```

> If you are using a virtual environment, activate it before running the command.

---

## 🛠️ ROS2 Build Instructions

```bash
cd ~/cobot1
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Recommended Execution Order

### 1️⃣ Terminal A — Launch ROS2

```bash
cd ~/cobot1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ramen_chef chef.launch.py
```

> If `chef.launch.py` requires arguments (e.g., robot name, host, port, mode, model), pass them according to the launch file definition.

---

### 2️⃣ Terminal B — Run Flask Web UI

Sourcing the ROS environment is recommended as the web UI communicates with ROS topics.

```bash
cd ~/cobot1/flask_recovery
source /opt/ros/humble/setup.bash
source ~/cobot1/install/setup.bash

python3 app.py
```

### 3️⃣ Browser Access

Open your browser and navigate to:

* 🔗 Default address: `http://localhost:5000`

---

## 📝 Quick Start Summary

**Terminal A**

```bash
cd ~/cobot1 && source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch ramen_chef chef.launch.py
```

**Terminal B**

```bash
cd ~/cobot1/flask_recovery && source /opt/ros/humble/setup.bash
source ~/cobot1/install/setup.bash
python3 app.py
```
