# RADO Mission Control System (v2.0)

## 📖 Overview
The **Reconnaissance and Autonomous Delivery Operation (RADO)** control system is a modular, web-based interface for controlling the rover.

Unlike traditional ROS GUIs (like RQT), this system uses a **Decoupled Architecture**. The Frontend (Web GUI) connects to the Backend (ROS 2) via WebSockets (`rosbridge_suite`), ensuring zero-latency video streaming and real-time telemetry without blocking the rover's core logic.

### 🏗️ Architecture
* **Frontend:** HTML5, CSS3, JavaScript (Leaflet.js for maps, ROSLibJS for communication).
* **Middleware:** `rosbridge_server` (WebSocket Port 9090) and `web_video_server` (HTTP Port 8080).
* **Backend (ROS 2 Humble):**
    * `state_manager_node`: Multiplexer for Manual vs. Autonomous control.
    * `system_monitor_node`: Watchdog for network health (Jetson/Raspi) and error logging.
    * `coordinate_follower_node`: The autonomous navigation brain.

---

## 🚀 Getting Started

### Prerequisites
* **OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **ROS Version:** ROS 2 Humble (Desktop Full)
* **Hardware:**
    * Pixhawk (via MAVROS)
    * ZED Camera (via ZED SDK/Wrapper)
    * Joystick/Gamepad

### Installation

#### 1. Clone the Repository
Navigate to your ROS 2 workspace `src` directory and clone this repo.

```bash
cd ~/ros2_ws/src
git clone <link_of_this_repo>
```

#### 2. Install Dependencies
We use standard ROS tools (rosdep) to install all system dependencies automatically (including rosbridge, web_video_server, and Python libraries).

```bash
cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src -y --ignore-src
```

#### 3. Install Additional Dependencies

```bash
# Install rosbridge suite
sudo apt install ros-humble-rosbridge-suite

# Install web video server
sudo apt install ros-humble-web-video-server

# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install ZED SDK (follow official instructions from Stereolabs)
# https://www.stereolabs.com/developers/release/

```

#### 4. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select rado_control_3
source install/setup.bash
```

---

## 🕹️ How to Run

The system requires two terminals (one for the backend, one for the frontend server).

### Terminal 1: The Backend (ROS)
This single launch file starts the ROS Bridge, Video Server, System Monitor, and all Control Nodes.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rado_control_3 rado_mission.launch.py
```

**What this launches:**
- `rosbridge_server` on WebSocket port 9090
- `web_video_server` on HTTP port 8080
- `state_manager_node` - Control multiplexer
- `system_monitor_node` - Health watchdog
- `coordinate_follower_node` - Autonomous navigation

### Terminal 2: The Frontend (GUI)
We use Python's built-in HTTP server to serve the web files to your browser.

```bash
cd ~/ros2_ws/src/rado_control_3/web_gui
python3 -m http.server 8000
```

### Accessing the GUI
1. Open a browser (Chrome/Firefox recommended)
2. Navigate to:
   - **Local:** `http://localhost:8000`
   - **Remote:** `http://[ROVER_IP]:8000` (if accessing from another device)

---

## 🗺️ Key Features

### 1. Reconnaissance Mode (Manual)
- **Live Feed:** Real-time ZED camera stream via `web_video_server`
- **Logging:** Select an object type and color, then click **LOG STEP**. This saves the rover's current GPS position to the mission plan file on the rover.
- **Map:** Satellite view of the rover's position using Leaflet.js

### 2. Mission Mode (Autonomous)
- **Switch:** Use the **PROCEED** button to engage the autonomous brain
- **Logic:** The rover reads the mission file and executes the logged coordinates sequentially
- **Safety:** The **EMERGENCY STOP** button immediately revokes control from the autonomous node and returns it to the joystick

### 3. System Health
- **Network:** Continuously pings subsystems (Jetson/Raspi) and displays status dots (Green/Red)
- **Topics:** Monitors critical ROS topics (GPS, Motors) for data flow timeouts

---

## 📁 Project Structure

```
rado_control_3/
├── launch/
│   └── rado_mission.launch.py       # Main launch file
├── nodes/
│   ├── state_manager_node.py        # Control multiplexer
│   ├── system_monitor_node.py       # Health monitoring
│   └── coordinate_follower_node.py  # Autonomous navigation
├── web_gui/
│   ├── index.html                   # Main GUI interface
│   ├── css/
│   │   └── styles.css               # GUI styling
│   ├── js/
│   │   ├── ros_connection.js        # ROSLibJS WebSocket handler
│   │   ├── map_manager.js           # Leaflet map integration
│   │   └── telemetry_display.js     # Real-time data display
│   └── assets/                      # Images and icons
├── config/
│   ├── params.yaml                  # ROS parameters
│   └── mission_plan.json            # Logged waypoints
├── package.xml
├── setup.py
└── README.md
```

---

## 🔧 Configuration

## 🎮 Controls

### Reconnaissance Mode
- **Joystick:** Manual control enabled
- **LOG STEP Button:** Records current position with metadata
- **Camera Feed:** Live view from ZED camera

### Mission Mode
- **PROCEED Button:** Starts autonomous execution
- **EMERGENCY STOP:** Immediately halts autonomous control
- **Progress Indicator:** Shows current waypoint execution

---

## 🐛 Troubleshooting

### Issue: GUI not connecting to ROS
**Solution:**
```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Test WebSocket connection
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" http://localhost:9090
```

### Issue: Video stream not showing
**Solution:**
```bash
# Verify web_video_server is running
ros2 node list | grep web_video_server

# Check available streams
curl http://localhost:8080/stream_viewer?topic=/zed/zed_node/rgb/image_rect_color
```

### Issue: GPS data not updating
**Solution:**
```bash
# Check MAVROS connection
ros2 topic echo /mavros/global_position/global

# Verify Pixhawk connection
ros2 run mavros mavsys status
```

### Issue: Emergency stop not working
**Solution:**
- Verify the emergency stop topic is being published
- Check state_manager_node logs for errors


### Tested Configurations
- ✅ Jetson Xavier NX + Ubuntu 22.04 + ROS 2 Humble
- ✅ Raspberry Pi 4 (8GB) + Ubuntu 22.04 + ROS 2 Humble
- ✅ Desktop PC + Ubuntu 22.04 + ROS 2 Humble

---

