# RADO Mission Control System

The **RADO (Reconnaissance and Autonomous Delivery Operation)** Mission Control System provides a complete, real-time Ground Control Station (GCS) for your rover. It runs a web-based interface that communicates with a ROS 2 backend to manage autonomous missions, GPS tracking, and live video streaming.

> This project is designed for real hardware — GPS from **MAVROS** and video from a **ZED camera**.



## Features

* **Web-Based GCS** — Control your rover from any device on the same network (laptop, tablet, phone) via a web browser.
* **Live Map Display** — Real-time satellite map showing the rover position (from MAVROS) and the path history.
* **Live Video Feed** — Streams ZED camera feed to the browser (topic: `/zed/zed_node/rgb/image_rect_color`).
* **Reconnaissance Mode (MANUAL)** — Drive with a joystick and log GPS coordinates with object & color labels to `mission_plan.txt`.
* **Mission Mode (AUTONOMOUS)** — Execute logged waypoints sequentially using the coordinate follower node.
* **Secure Controls** — Token-based authentication to prevent unauthorized commands.


## Prerequisites

Ensure the following are installed and functional on your Ubuntu machine:

* ROS 2 **Humble** (Desktop install)
* A ROS 2 workspace (for example `~/ros2_ws`)
* **MAVROS** running and connected to a Pixhawk (or compatible autopilot)

---

## Installation

This repository should contain two folders at the root:

* `rado_control_3` (ROS 2 package)
* `flask_gcs` (Flask web app)

### 1. Clone the repository

```bash
cd ~
git clone https://github.com/genzcoder001/Main_Control.git
```


### 2. Organize folders

```bash
# Move ROS 2 package into your workspace's src
mv ~/Main_Control/rado_control_3 ~/ros2_ws/src/

# Move Flask app to your home directory
mv ~/Main_Control/flask_gcs ~/

# Remove the top-level repo folder if desired
rm -rf ~/Main_Control
```

### 3. Install system & ROS dependencies

```bash
sudo apt update

# Install Flask and OpenCV
sudo apt install python3-flask python3-opencv

# From your ROS 2 workspace, install ROS dependencies listed in package.xml
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
```

### 4. Build the ROS 2 workspace

```bash
cd ~/ros2_ws
colcon build --packages-select rado_control_3
```

---

## How to Run

You must run three separate terminals (or tmux windows) for full functionality.

### Terminal 1 — Launch MAVROS

Start MAVROS with the appropriate FCU URL for your setup:

```bash
ros2 launch mavros mavros.launch.py fcu_url:=<your_fcu_url>
```

### Terminal 2 — Launch the ROS 2 control nodes

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rado_control_3 rado_mission.launch.py
```

This launches the `state_manager_node`, `coordinate_follower_node`, joystick nodes, and any other control nodes defined in the launch file.

### Terminal 3 — Launch the Flask GCS

**Important:** source the workspace here as well so Flask can use ROS 2 packages if needed.

```bash
cd ~/ros2_ws
source install/setup.bash

cd ~/flask_gcs
python3 app.py
```

You should see the Flask server start on `http://0.0.0.0:5000`.

---

## Access the GUI

Find the VM's or machine's IP address (example shown with `ip a`):

```bash
ip a
```

From another device on the same network open in your browser:

```
http://<YOUR_MACHINE_IP>:5000
```

Example: `http://192.168.64.3:5000`

---



## File Structure

```
repo/
├── rado_control_3/      # ROS 2 backend package
└── flask_gcs/           # Flask-based Ground Control Station
```

---




