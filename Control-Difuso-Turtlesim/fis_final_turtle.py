#!/usr/bin/env python

import rclpy  # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node  # Node class for Python nodes
from geometry_msgs.msg import Twist  # Twist (linear and angular velocities) message class
from turtlesim.msg import Pose  # Turtlesim pose (x, y, theta) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # Quality of Service (tune communication between nodes)
from math import sqrt, atan2, pow, pi  # Common mathematical functions
import numpy as np
import yaml  # Module for working with YAML files
from scipy.interpolate import interp2d

# Node class
class RobotController(Node):

    def __init__(self):
        info = '''\nMake the robot go to the specified goal.\n'''
        print(info)
        super().__init__('robot_controller')  # Create a node with name 'robot_controller'
        qos_profile = QoSProfile(  # Quality of Service profile
            reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable (not best effort) communication
            history=QoSHistoryPolicy.KEEP_LAST,  # Keep/store only up to last N samples
            depth=10  # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_ctrl_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)  # Publisher which will publish Twist message to the topic '/turtle1/cmd_vel' adhering to 'qos_profile' QoS profile
        self.robot_pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.robot_feedback_callback, qos_profile)  # Subscriber which will subscribe Pose message from the topic '/turtle1/pose' and execute 'robot_feedback_callback()' callback function adhering to 'qos_profile' QoS profile
        self.robot_pose_sub2 = self.create_subscription(Pose, '/turtle2/pose', self.robot2_feedback_callback, qos_profile)  # Subscriber for turtle2 pose
        self.robot_ctrl_pub2 = self.create_publisher(Twist, '/turtle2/cmd_vel', qos_profile)  # Publisher for turtle2 commands
        timer_period = 0.1  # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback)  # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds

        # Initialize variables
        self.robot_pose = Pose()  # Robot pose (position & rotation)
        self.robot_pose2 = Pose()  # Robot2 pose (position & rotation)
        self.robot_flag = False  # Flag to check if robot feedback is available
        self.goal_pose = Pose()  # Goal pose (position & rotation)
        self.goal_flag = False  # Flag to check if set goal is reached
        self.ctrl_msg = Twist()  # Robot control commands (twist)
        self.ctrl_msg2 = Twist()  # Robot2 control commands (twist)
        (self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta) = 5.5, 1.0, 0.0
        self.goal_count = 0
        self.movement_type = 'linear'
        self.current_sequence = 0

        # Cargar parámetros desde archivo YAML
        yaml_path = '/home/francisco/ros2_ws/src/move_turtle_pkg/move_turtle_pkg/fis_control_surface.yaml'
        self.load_params_from_yaml(yaml_path)
        
        self.interp_l = interp2d(self.X, self.Y, self.Z_l, kind='linear')
        self.interp_a = interp2d(self.X, self.Y, self.Z_a, kind='linear')
    
    def load_params_from_yaml(self, filename):
        # Load control surfaces
        with open(filename, 'r') as file:
            control_data = yaml.safe_load(file)
            self.X = np.array(control_data['robot_controller']['ros__parameters']['X'])
            self.Y = np.array(control_data['robot_controller']['ros__parameters']['Y'])
            self.Z_l = np.array(control_data['robot_controller']['ros__parameters']['Z_l'])
            self.Z_a = np.array(control_data['robot_controller']['ros__parameters']['Z_a'])

    def robot_feedback_callback(self, message):
        '''Robot feedback (pose) callback'''
        self.robot_pose = message  # Capture incoming message (Pose)
        self.robot_flag = True  # Set robot flag to feedback available

    def robot2_feedback_callback(self, message):
        '''Robot2 feedback (pose) callback'''
        self.robot_pose2 = message  # Capture incoming message (Pose)

    def robot_controller_callback(self):
        '''Robot controller (twist) callback'''
        if self.robot_flag:
            lin_vel, ang_vel = self.set_robot_control(self.movement_type)  # Set and publish robot controls
            self.ctrl_msg.linear.x = lin_vel  # Set robot linear velocity
            self.ctrl_msg.angular.z = ang_vel  # Set robot angular velocity
            self.robot_ctrl_pub.publish(self.ctrl_msg)  # Publish robot controls message

            # Follow logic for turtle2
            da = self.get_position_error_turtle2()
            
            # Adjust the angular velocity for turtle2 to follow turtle1
            angle_to_turtle1 = atan2(self.robot_pose.y - self.robot_pose2.y, self.robot_pose.x - self.robot_pose2.x)
            angle_diff = angle_to_turtle1 - self.robot_pose2.theta
            angle_diff = round(atan2(np.sin(angle_diff), np.cos(angle_diff)), 2)
            
            (vel_l, vel_a) = self.fis(da, angle_diff)
            #self.ctrl_msg2.linear.x = 0.0 # Para Tarea 1
            self.ctrl_msg2.linear.x = vel_l # Para Tarea 2
            self.ctrl_msg2.angular.z = vel_a
            
            self.robot_ctrl_pub2.publish(self.ctrl_msg2)

    def get_position_error(self):
        '''Error in position as Euclidean distance between current pose and goal pose.'''
        return sqrt(pow((self.goal_pose.x - self.robot_pose.x), 2) + pow((self.goal_pose.y - self.robot_pose.y), 2))

    def get_position_error_turtle2(self):
        '''Error in position as Euclidean distance between turtle1 and turtle2.'''
        error2 = sqrt(pow((self.robot_pose.x - self.robot_pose2.x), 2) + pow((self.robot_pose.y - self.robot_pose2.y), 2))
        return round(error2, 2)

    def get_rotation_error(self):
        '''Error in rotation as relative angle between current pose and goal pose.'''
        desired_angle = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
        dtheta = desired_angle - self.robot_pose.theta + 2 * pi
        if abs(desired_angle - self.robot_pose.theta) < abs(desired_angle - self.robot_pose.theta + 2 * pi):
            dtheta = desired_angle - self.robot_pose.theta
        return dtheta

    def get_linear_velocity(self, gain=1.0):
        '''Compute robot linear velocity using P-controller'''
        return gain * self.get_position_error()

    def get_angular_velocity(self, gain=1.0):
        '''Compute robot angular velocity using P-controller'''
        return gain * self.get_rotation_error()

    def set_robot_control(self, movement_type):
        rot_tol = 0.005
        pos_tol = 0.05
        if movement_type == 'angular':
            if abs(self.get_rotation_error()) > rot_tol:  # Orient at goal
                lin_vel = 0.0  # Set robot linear velocity
                ang_vel = self.get_angular_velocity(gain=6.0)  # Set robot angular velocity
                return lin_vel, ang_vel
            else:
                self.set_robot_sequence()
        if movement_type == 'linear':
            if self.get_position_error() > pos_tol:  # Go to goal
                lin_vel = self.get_linear_velocity(gain=2.0)  # Set robot linear velocity
                ang_vel = 0.0
                return lin_vel, ang_vel
            else:
                self.set_robot_sequence()

        # Stop robot after the goal is reached
        lin_vel = 0.0  # Set robot linear velocity
        ang_vel = 0.0  # Set robot angular velocity
        return lin_vel, ang_vel

    def set_robot_sequence(self):
        self.seq_data = [['angular', 10.0, 5.0, 0.0],
                         ['linear', 10.0, 5.0, 0.0],
                         ['angular', 5.5, 9.5, 0.0],
                         ['linear', 5.5, 9.5, 0.0],
                         ['angular', 1.0, 5.0, 0.0],
                         ['linear', 1.0, 5.0, 0.0],
                         ['angular', 5.5, 1.0, 0.0],
                         ['linear', 5.5, 1.0, 0.0],
                         ['angular', 1.0, 5.0, 0.0], ]
        
        self.seq_data2 = [['angular', 1.0, 5.0, 0.0],
                         ['linear', 1.0, 5.0, 0.0],
                         ['angular', 5.5, 9.5, 0.0],
                         ['linear', 5.5, 9.5, 0.0],
                         ['angular', 10.0, 5.0, 0.0],
                         ['linear', 10.0, 5.0, 0.0],
                         ['angular', 5.5, 1.0, 0.0],
                         ['linear', 5.5, 1.0, 0.0],
                         ['angular', 10.0, 5.0, 0.0], ]
        
        if self.current_sequence == 0:
            seq = self.seq_data
        else:
            seq = self.seq_data2

        if self.goal_count < len(self.seq_data):
            (self.movement_type, self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta) = seq[self.goal_count]
            print(self.movement_type, self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta)
            self.goal_count += 1
            print("goal count: ", self.goal_count)
            return self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta
        if self.goal_count == len(self.seq_data):
            self.goal_count = 0
            self.current_sequence = 1 - self.current_sequence
            
    def fis(self, error_d, error_a):
        vel_l = self.interp_l(error_d, error_a)[0]
        vel_a = self.interp_a(error_d, error_a)[0]
        return vel_l, vel_a

def main(args=None):
    '''Initialize and run the node'''
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)  # Keep the node alive (until it's shut down externally)
    robot_controller.destroy_node()  # Release resources
    rclpy.shutdown()  # Shutdown the node


if __name__ == '__main__':
    main()
