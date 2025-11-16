#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import threading
import time
from flask import Flask, render_template, jsonify, request, Response

SECRET_TOKEN = "RADO_2026_SECURE_ACCESS"

class GcsNode(Node):
    def __init__(self):
        super().__init__('gcs_web_node')
        self.current_lat = 15.3911  # BITS Goa default
        self.current_lon = 73.8782
        self.current_state = "UNKNOWN"
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # File Setup
        flask_dir = os.path.dirname(os.path.realpath(__file__))
        self.mission_file_path = os.path.join(flask_dir, 'mission_plan.txt')

        # --- PUBLISHERS ---
        self.command_publisher_ = self.create_publisher(String, '/gcs/command', 10)
        
        # --- SUBSCRIBERS ---
        # 1. State Subscriber (To show status in GUI)
        self.create_subscription(String, '/rover_state', self.state_callback, 10)

        # 2. If you have a camera feeding through ROS (ZED), uncomment below:
        # self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)

        # 3. We DO NOT simulate GPS here anymore (commented out)
        # self.fake_lat = 15.3911 
        # self.fake_lon = 73.8782 
        # self.gps_sim_timer = self.create_timer(1.0, self.simulate_gps_callback)
        
        # 4. Camera Simulation (Fake Video) [commented out the timer to stop sim]
        # self.video_timer = self.create_timer(0.033, self.simulate_video_callback)
        self.sim_circle_x = 0

    def state_callback(self, msg):
        self.current_state = msg.data

    # Simulation helpers — kept for debugging but NOT active by default
    def simulate_gps_callback(self):
        self.fake_lat += 0.00005
        self.fake_lon += 0.00005
        self.current_lat = self.fake_lat
        self.current_lon = self.fake_lon
    # only for simulation
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def simulate_video_callback(self):
        """Generates a fake video feed (bouncing red ball)."""
        img = np.zeros((480, 640, 3), np.uint8)
        self.sim_circle_x = (self.sim_circle_x + 5) % 640
        cv2.circle(img, (self.sim_circle_x, 240), 50, (0, 0, 255), -1)
        cv2.putText(img, "NO CAMERA - SIMULATION", (150, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        ret, buffer = cv2.imencode('.jpg', img)
        self.latest_frame = buffer.tobytes()

    def image_callback(self, msg):
        """Handles real camera frames from ROS (ZED)."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, buffer = cv2.imencode('.jpg', cv_image)
            self.latest_frame = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def publish_command(self, cmd):
        msg = String(data=cmd)
        self.command_publisher_.publish(msg)
        self.get_logger().info(f'GUI published command: {cmd}')

    def log_mission_step(self, object_type, color_type):
        # Check for no-GPS case -> small tolerance
        if abs(self.current_lat) < 1e-9 and abs(self.current_lon) < 1e-9:
            return "FAILED (No GPS)"
        line = f"{object_type},{color_type},{self.current_lat:.7f},{self.current_lon:.7f}\n"
        try:
            with open(self.mission_file_path, 'a') as f: f.write(line)
            return f"Logged {object_type}, {color_type}"
        except Exception as e:
            self.get_logger().error(f"Mission file write error: {e}")
            return "FAILED (Permissions)"

# --- Flask App ---
app = Flask(__name__)
rclpy.init()
ros_node = GcsNode()

@app.route('/')
def index(): return render_template('index.html')

@app.route('/status_update', methods=['GET'])
def status_update():
    """Returns GPS and State in one call."""
    return jsonify({
        'lat': ros_node.current_lat,
        'lon': ros_node.current_lon,
        'state': ros_node.current_state
    })

def generate_video():
    """Generator function for the video stream."""
    while True:
        frame = ros_node.latest_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03) # Approx 30 FPS cap

@app.route('/video_feed')
def video_feed():
    """Route for the video stream <img> tag."""
    return Response(generate_video(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/log_data', methods=['POST'])
def log_data():
    if request.headers.get('X-Auth-Token') != SECRET_TOKEN: return jsonify({'message': 'Auth Failed'}), 403
    data = request.json
    return jsonify({'message': ros_node.log_mission_step(data.get('object_type'), data.get('color_type'))})

@app.route('/mission_command', methods=['POST'])
def mission_command():
    if request.headers.get('X-Auth-Token') != SECRET_TOKEN: return jsonify({'message': 'Auth Failed'}), 403
    ros_node.publish_command(request.json.get('command'))
    return jsonify({'message': 'Sent'})

def ros_spin():
    try:
        # subscribe to mavros global position so GUI node updates lat/lon if needed
        # If MAVROS node is running in the same ROS graph, uncomment the subscription below:
        # ros_node.create_subscription(NavSatFix, '/mavros/global_position/global', lambda msg: ros_node.gps_callback(msg), 10)
        rclpy.spin(ros_node)
    except Exception:
        pass
    finally:
        try:
            ros_node.destroy_node()
        except Exception:
            pass

if __name__ == '__main__':
    threading.Thread(target=ros_spin, daemon=True).start()
    # Threaded so Flask can handle multiple concurrent requests while ROS spins
    app.run(host='0.0.0.0', port=5000, threaded=True)
