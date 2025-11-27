# RADO Mission Control System (v2.0)

## 📖 Overview
The **Reconnaissance and Autonomous Delivery Operation (RADO)** control system is a modular, web-based interface for controlling the rover.

Unlike traditional ROS GUIs (like RQT), this system uses a **Decoupled Architecture**. The Frontend (Web GUI) connects to the Backend (ROS 2) via WebSockets (`rosbridge_suite`), ensuring zero-latency video streaming and real-time telemetry without blocking the rover's core logic.

### 🏗️ Architecture
* **Frontend:** HTML5, CSS3, JavaScript (Leaflet.js for maps, ROSLibJS for communication).
* **Middleware:** `rosbridge_server` (WebSocket Port 9090) and `web_video_server` (HTTP Port 8080).
* **Backend (ROS 2 Humble):**
    * `state_manager_node`: Multiplexer for Manual vs. Autonomous control.
    * `system_monitor_node`: Watchdog for network health and error logging.
    * `coordinate_follower_node`: The autonomous navigation brain.

---


### Prerequisites
* Ubuntu 22.04
* ROS 2 Humble (Desktop Full)
* Python 3.10+

---


