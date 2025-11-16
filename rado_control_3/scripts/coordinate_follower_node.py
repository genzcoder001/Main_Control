#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import NavSatFix 
from ament_index_python.packages import get_package_share_directory
import os
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('coordinate_follower')
        self.internal_state = 'WAITING_FOR_PROCEED' # Start in a waiting state
        self.mission_goals = []
        self.current_goal_index = -1
        
        # --- SIMULATION TIMER ---
        self.drive_start_time = 0.0
        self.drive_duration = 5.0 # Drive for 5 seconds
        
        self.has_gps_fix = False # Waits for real MAVROS data

        home_dir = os.path.expanduser('~')
        self.mission_file_path = os.path.join(home_dir, 'flask_gcs', 'mission_plan.txt')
        
        # --- PUBLISHERS ---
        self.task_complete_pub = self.create_publisher(Bool, '/auto/task_complete', 10)
        self.velocity_pub = self.create_publisher(Twist, '/auto/cmd_vel', 10) # Publishes to the AUTO topic
        self.gcs_command_pub = self.create_publisher(String, '/gcs/command', 10)
        
        # --- SUBSCRIBERS ---
        self.create_subscription(String, '/rover_state', self.state_callback, 10)
        self.create_subscription(String, '/gcs/command', self.gcs_command_callback, 10)
        
        # This node LISTENS to the real MAVROS GPS topic
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        
        self.load_mission()
        if not self.mission_goals:
            self.get_logger().warn('No mission goals found. Waiting for logs.')
        
        self.get_logger().info('Coordinate Follower (Dummy Brain) is running.')
        self.get_logger().info('Waiting for first GPS fix from MAVROS...')


    def gps_callback(self, msg):
        """Confirms that MAVROS is running and we have a GPS fix."""
        if not self.has_gps_fix:
            self.get_logger().info('GPS Fix Acquired from MAVROS!')
            self.has_gps_fix = True

    def load_mission(self):
        self.get_logger().info(f'Loading mission plan from {self.mission_file_path}')
        try:
            self.mission_goals = [] 
            with open(self.mission_file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) == 4: 
                        try:
                            lat = float(parts[2])
                            lon = float(parts[3])
                            self.mission_goals.append(Point(x=lat, y=lon))
                        except ValueError: pass
            self.get_logger().info(f'Loaded {len(self.mission_goals)} goals.')
        except Exception as e:
            self.get_logger().error(f'Failed to load mission file: {e}')

    def gcs_command_callback(self, msg):
        command = msg.data.upper()
        if command == 'PROCEED' and self.internal_state == 'WAITING_FOR_PROCEED':
            self.load_mission() # Reload in case user added points
            self.request_next_goal()

    def request_next_goal(self):
        self.current_goal_index += 1
        if self.current_goal_index < len(self.mission_goals):
            goal = self.mission_goals[self.current_goal_index]
            self.get_logger().info(f'Proceeding to Goal {self.current_goal_index + 1}: ({goal.x}, {goal.y})')
            self.gcs_command_pub.publish(String(data="PROCEED"))
        else:
            self.get_logger().info('Mission Complete. Waiting for new goals.')
            self.internal_state = 'WAITING_FOR_PROCEED'
            self.current_goal_index = len(self.mission_goals) - 1

    def state_callback(self, msg):
        rover_state = msg.data.upper()
        if rover_state == 'AUTONOMOUS' and self.internal_state == 'WAITING_FOR_PROCEED':
            if not self.has_gps_fix:
                self.get_logger().error("Cannot start autonomous: No GPS Fix!")
                self.gcs_command_pub.publish(String(data="MANUAL")) # Instantly go back to manual
                return
            
            self.internal_state = 'AUTONOMOUS_ACTIVE'
            self.drive_start_time = time.time()
        elif rover_state == 'MANUAL' and self.internal_state == 'AUTONOMOUS_ACTIVE':
            self.internal_state = 'WAITING_FOR_PROCEED'

    def control_loop(self):
        # This node ONLY runs its logic when in autonomous mode
        if self.internal_state != 'AUTONOMOUS_ACTIVE':
            return

        elapsed = time.time() - self.drive_start_time
        
        if elapsed < self.drive_duration:
            # --- DUMMY AUTONOMOUS LOGIC ---
            # For 5 seconds, just command the rover to drive forward
            cmd = Twist()
            cmd.linear.x = 0.5 # Drive forward at 0.5 m/s
            self.velocity_pub.publish(cmd)
        else:
            # --- GOAL FINISHED ---
            self.get_logger().info('Arrived at goal (5-second sim complete).')
            self.velocity_pub.publish(Twist()) # Stop
            self.task_complete_pub.publish(Bool(data=True)) # Signal "I'm done"

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()