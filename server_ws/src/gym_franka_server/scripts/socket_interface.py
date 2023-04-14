#! /usr/bin/env python3
import rospy
import socket
import numpy as np
import pyquaternion as pqt

from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from franka_msgs.msg import ErrorRecoveryActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String


BUFFER_SIZE = 1024


class SocketInterface:
    def __init__(self):
        self.status_subscriber = rospy.Subscriber('/cartesian_impedance_controller/robot_mode', String, self.update_robot_mode)

        self.pose_publisher = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=1)
        self.gripper_publisher = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=1)
        self.grasp_publisher = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
        self.grasp_cancel_publisher = rospy.Publisher('/franka_gripper/grasp/cancel', GoalID, queue_size=1)
        self.error_recovery_publisher = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)

        self.init_position = np.array([0.3, 0, 0.5])
        self.init_rotation = pqt.Quaternion(0, -1, 0, 0)

        self.position = None
        self.rotation = None
        self.start_input_position = None
        self.start_input_rotation = None
        self.robot_mode = 0

        self.gripper_goal = 0.04
        self.active_grasp_id = None

        # self.dynamic_reconfigure_client = Client('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node')
        # self.dynamic_reconfigure_client.update_configuration({'positional_stiffness': 150,
        #                                                       'rotational_stiffness': 20,
        #                                                       'nullspace_stiffness': 0})

        if rospy.get_param('/socket_interface/client') == 'iOS':
            self.position_gain = 4
            self.rotation_gain = 4
            self.coordinate_change = np.array([[0, 0, -1],
                                               [-1, 0, 0],
                                               [0, 1, 0]])
        else:
            self.position_gain = 1
            self.rotation_gain = 1
            self.coordinate_change = np.eye(3)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(0.1)
        self.socket.bind((rospy.get_param('/socket_interface/address'), int(rospy.get_param("/socket_interface/port"))))
        self.socket.listen(1)
        while not rospy.is_shutdown():
            try:
                self.connection, self.address = self.socket.accept()
                rospy.loginfo(f'[Connected] Socket established: {self.address}')
                return
            except socket.timeout:
                pass

    def update_robot_mode(self, msg:String):
        self.robot_mode = int(msg.data.split(' ')[1])

    def process_command(self):
        try:
            data = self.connection.recv(BUFFER_SIZE)
            if len(data) == 0:
                rospy.signal_shutdown("[Socket Interface] Connection lost.")
        except ConnectionResetError as e:
            rospy.signal_shutdown("[Socket Interface] Connection reset.")
        except socket.timeout:
            return

        data_strings = data.decode('utf8').split(' ')

        command = data_strings[0]
        try:
            params = np.array([float(s) for s in data_strings[1:]])
        except ValueError as e:
            print(f'Failed to parse command: {data_strings}')
            return

        if command == '<Start>':
            self.start_input_position = params[:3]
            self.start_input_rotation = pqt.Quaternion(array=params[3:7])
        elif command == '<Track>':
            input_position = params[:3]
            input_rotation = pqt.Quaternion(array=params[3:7])
            
            delta_position = (input_position - self.start_input_position) @ self.coordinate_change.T * self.position_gain
            self.position = self.init_position + delta_position

            delta_rotation = input_rotation / self.start_input_rotation
            delta_rotation = pqt.Quaternion(delta_rotation.w, *(delta_rotation.vector @ self.coordinate_change.T)) ** self.rotation_gain
            self.rotation = delta_rotation * self.init_rotation

            self.pose_publisher.publish(self.create_pose_command())
            if len(params) > 7:
                self.move_gripper(params[7])

        elif command == '<Gripper>':
            self.move_gripper(params[0])
        elif command == '<Recovery>':
            self.publish_error_recovery()
        elif command == '<Close>':
            rospy.signal_shutdown("[Socket Interface] Connection closed.")
        elif command == '<Mode>':
            timestamp = data_strings[1]
            response = f'<Mode> {self.robot_mode} {timestamp}'
            self.connection.send(response.encode('utf8'))

    def move_gripper(self, action):
        if action > 0 and self.gripper_goal != 0.04:
            self.gripper_goal = 0.04
            self.publish_gripper_command()
        elif action < 0 and self.gripper_goal != 0.005:
            self.gripper_goal = 0.005
            self.publish_gripper_command()

    def create_pose_command(self):
        pose_command = PoseStamped()
        pose_command.pose.position.x = self.position[0]
        pose_command.pose.position.y = self.position[1]
        pose_command.pose.position.z = self.position[2]
        pose_command.pose.orientation.w = self.rotation.w
        pose_command.pose.orientation.x = self.rotation.x
        pose_command.pose.orientation.y = self.rotation.y
        pose_command.pose.orientation.z = self.rotation.z
        return pose_command

    def publish_error_recovery(self):
        recovery_command = ErrorRecoveryActionGoal()
        self.error_recovery_publisher.publish(recovery_command)

    def publish_gripper_command(self):
        if self.gripper_goal > 0.035:
            if self.active_grasp_id is not None:
                self.grasp_cancel_publisher.publish(self.active_grasp_id)
                self.active_grasp_id = None
            move_command = MoveActionGoal()
            move_command.goal.width = 0.08
            move_command.goal.speed = 0.1
            self.gripper_publisher.publish(move_command)

        else:
            grasp_command = GraspActionGoal()
            grasp_command.goal.width = self.gripper_goal
            grasp_command.goal.force = 0.5
            grasp_command.goal.speed = 0.2
            grasp_command.goal.epsilon.inner = 0.05
            grasp_command.goal.epsilon.outer = 0.05
            self.active_grasp_id = grasp_command.goal_id
            self.grasp_publisher.publish(grasp_command)

if __name__ == '__main__':
    rospy.init_node('socket_interface')
    rospy.loginfo('[Init] Socket Interface')
    socket_interface = SocketInterface()
    while not rospy.is_shutdown():
        socket_interface.process_command()
