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

### Software
* **Ubuntu 22.04**
* **ROS2 Humble**
* **Python 3.10+**

### Hardware
* **Doosan M0609**
* **Onrobot RG2 Gripper**

> ⚠️ **Note**: To run this on a real robot or simulator, the corresponding bringup packages and drivers must be installed separately.
> * Bringup packages: [doosan-robot2 GitHub](https://github.com/doosan-robotics/doosan-robot2.git)

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
cd ~/cobot1/src
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Recommended Execution Order

### 1. Terminal A — Launch ROS2

```bash
ros2 launch ramen_chef chef.launch.py
```

### 2. Terminal B — Run Flask Web UI

Sourcing the ROS environment is recommended as the web UI communicates with ROS topics.

```bash
cd ~/cobot1/flask_recovery
source /opt/ros/humble/setup.bash
source ~/cobot1/install/setup.bash

python3 app.py
```

### 3. Browser Access

Open your browser and navigate to:

* 🔗 Default address: `http://localhost:5000`

---

## 📜 License

This project is licensed under the **Apache License 2.0**.  
Feel free to use, modify, and distribute this software under the terms of the Apache License.
