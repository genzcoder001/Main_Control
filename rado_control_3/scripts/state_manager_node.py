#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.state = 'MANUAL'
        self.manual_twist_msg = None
        self.auto_twist_msg = None
        
        # Publisher to the final motor command topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher to inform other nodes of the current state
        self.state_pub = self.create_publisher(String, '/rover_state', 10)
        
        # === NEW SUBSCRIBERS TO ACT AS A MUX ===
        # Subscriber to manual joystick commands- by default 0 velocity
        self.create_subscription(Twist, '/manual/cmd_vel', self.manual_cmd_callback, 10)
        # Subscriber to autonomous script commands
        self.create_subscription(Twist, '/auto/cmd_vel', self.auto_cmd_callback, 10)
        
        # Subscribers for state logic (unchanged)
        self.create_subscription(String, '/gcs/command', self.gcs_command_callback, 10)
        self.create_subscription(Bool, '/auto/task_complete', self.task_complete_callback, 10)
        
        # The main loop for publishing state and motor commands
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('State Manager (with Mux Logic) is running.')

    def manual_cmd_callback(self, msg):
        """Stores the latest command from the joystick."""
        self.manual_twist_msg = msg

    def auto_cmd_callback(self, msg):
        """Stores the latest command from the autonomous script."""
        self.auto_twist_msg = msg

    def gcs_command_callback(self, msg):
        """Callback for commands from the GCS."""
        command = msg.data.upper()
        
        # 'PROCEED' is the command to enter autonomous mode
        if command == 'PROCEED' and self.state == 'MANUAL':
            self.state = 'AUTONOMOUS'
            self.get_logger().info('State changed to: AUTONOMOUS')
            
        # 'MANUAL' is the emergency stop
        elif command == 'MANUAL':
            self.state = 'MANUAL'
            self.get_logger().info('Emergency stop: State changed to MANUAL')

    def task_complete_callback(self, msg):
        """Callback for the 'task complete' signal."""
        if self.state == 'AUTONOMOUS' and msg.data is True:
            self.get_logger().info('Task complete. Reverting to MANUAL.')
            self.state = 'MANUAL'

    def control_loop(self):
        """Publishes state and forwards the correct motor command."""
        # Publish the current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        # === THE MUX LOGIC ===
        if self.state == 'AUTONOMOUS':
            if self.auto_twist_msg:
                self.cmd_vel_pub.publish(self.auto_twist_msg)
        elif self.state == 'MANUAL':
            # In manual mode, the rover should not move unless a command is sent.
            # Your teleop node should publish zero-velocity twists when idle.
            if self.manual_twist_msg:
                self.cmd_vel_pub.publish(self.manual_twist_msg)
            else:
                # If no joystick messages ever received, ensure we are stopped.
                self.cmd_vel_pub.publish(Twist())
        else:
            # If in some unknown state, stop the robot for safety.
            self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()