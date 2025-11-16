#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Twist 
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
        # These are now populated ONLY by the real MAVROS
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_state = "WAITING..." # Default state
        self.bridge = CvBridge()
        self.latest_frame = None
        
        flask_dir = os.path.dirname(os.path.realpath(__file__))
        self.mission_file_path = os.path.join(flask_dir, 'mission_plan.txt')
        self.get_logger().info(f'Writing logs to: {self.mission_file_path}')

        # --- PUBLISHERS ---
        # This node ONLY publishes commands from the GUI
        self.command_publisher_ = self.create_publisher(String, '/gcs/command', 10)
        
        # --- SUBSCRIBERS (REAL HARDWARE MODE) ---
        
        # 1. Listen to the rover's state from the state_manager
        self.create_subscription(String, '/rover_state', self.state_callback, 10)
        
        # 2. Listen to the REAL MAVROS GPS data
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        
        # 3. Listen to the REAL ZED Camera (Optional: change topic)
        self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)

        # 4. ALL SIMULATION TIMERS ARE REMOVED.
        
        self.get_logger().info('GcsNode running in REAL HARDWARE MODE.')
        self.get_logger().info('Listening to /rover_state, /mavros/global_position/global, and /zed/zed_node/rgb/image_rect_color')
        
    def state_callback(self, msg):
        """Stores the current rover state from the state_manager."""
        self.current_state = msg.data

    def gps_callback(self, msg):
        """Stores the current GPS data from the real MAVROS."""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
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
        if self.current_lat == 0.0: 
            return "FAILED (No GPS Fix)"
            
        line = f"{object_type},{color_type},{self.current_lat:.7f},{self.current_lon:.7f}\n"
        try:
            with open(self.mission_file_path, 'a') as f: 
                f.write(line)
            return f"Logged {object_type}, {color_type}"
        except Exception as e: 
            return "FAILED (File Error)"

# --- Flask Web Server (No changes from here down) ---
app = Flask(__name__)
rclpy.init()
ros_node = GcsNode()

@app.route('/')
def index(): return render_template('index.html')

@app.route('/status_update', methods=['GET'])
def status_update():
    return jsonify({
        'lat': ros_node.current_lat,
        'lon': ros_node.current_lon,
        'state': ros_node.current_state
    })

def generate_video():
    while True:
        frame = ros_node.latest_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        # If no real camera, we can send a "No Signal" frame
        else:
            img = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(img, "NO CAMERA SIGNAL", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            ret, buffer = cv2.imencode('.jpg', img)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
        time.sleep(0.03) # Limit to ~30fps

@app.route('/video_feed')
def video_feed():
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
    try: rclpy.spin(ros_node)
    except: pass
    finally: ros_node.destroy_node()

if __name__ == '__main__':
    threading.Thread(target=ros_spin).start()
    app.run(host='0.0.0.0', port=5000)