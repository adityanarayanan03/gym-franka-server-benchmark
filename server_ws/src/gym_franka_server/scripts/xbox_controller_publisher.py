#! /usr/bin/env python3
import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import GripperCommandActionGoal
from franka_gripper.msg import GraspActionGoal
from actionlib_msgs.msg import GoalID
from visualization_msgs.msg import InteractiveMarkerInit, InteractiveMarkerFeedback, InteractiveMarkerUpdate


CONTROL_GAIN = 0.01


class XboxControllerPublisher:
    def __init__(self):
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_parser, queue_size=1)
        self.pose_publisher = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)

        self.gripper_publisher = rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal, queue_size=1)
        self.grasp_publisher = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
        self.grasp_cancel_publisher = rospy.Publisher('/franka_gripper/grasp/cancel', GoalID, queue_size=1)

        self.x = 0.3
        self.y = 0
        self.z = 0.5

        self.gripper_goal = 0.04
        self.active_grasp_id = None

        self.dynamic_reconfigure_client = Client('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node')
        self.dynamic_reconfigure_client.update_configuration({'translational_stiffness': 250,
                                                              'rotational_stiffness': 15,
                                                              'nullspace_stiffness': 0})

    def joy_parser(self, msg:Joy):
        # Translation inputs
        left_analog_x = msg.axes[0]
        left_analog_y = msg.axes[1]
        left_trigger = msg.axes[2]
        right_trigger = msg.axes[5]

        # Rotational inputs
        right_analog_x = msg.axes[3]
        right_analog_y = msg.axes[4]

        # Homing inputs
        menu_button = msg.buttons[7]

        # Gripper inputs
        left_button = msg.buttons[4] == 1
        right_button = msg.buttons[5] == 1

        send_pose_update = False
        # Translation
        if abs(left_analog_x) > 0.05:
            send_pose_update = True
            self.y += left_analog_x * CONTROL_GAIN
        if abs(left_analog_y) > 0.05:
            send_pose_update = True
            self.x += left_analog_y * CONTROL_GAIN
        if 0 != left_trigger < 0.95  or 0 != right_trigger < 0.95:
            send_pose_update = True
            if left_trigger < right_trigger:
                travel = (1 - left_trigger) / 2
            else:
                travel = (right_trigger - 1) / 2
            self.z += travel * CONTROL_GAIN

        # Rotation
        # if 

        # Homing
        if menu_button:
            send_pose_update = True
            self.x = 0.3
            self.y = 0
            self.z = 0.5

        # Gripper
        if left_button or right_button:
            if left_button and self.gripper_goal != 0.04:
                self.gripper_goal = 0.04
                self.publish_gripper_command()
            elif right_button and self.gripper_goal != 0.005:
                self.gripper_goal = 0.005
                self.publish_gripper_command()
            
        if send_pose_update:
            pose_command = PoseStamped()
            pose_command.pose.position.x = self.x
            pose_command.pose.position.y = self.y
            pose_command.pose.position.z = self.z
            pose_command.pose.orientation.x = -1
            self.pose_publisher.publish(pose_command)

    def publish_gripper_command(self):
        if self.gripper_goal > 0.035:
            if self.active_grasp_id is not None:
                self.grasp_cancel_publisher.publish(self.active_grasp_id)
                self.active_grasp_id = None
            gripper_command = GripperCommandActionGoal()
            gripper_command.goal.command.position = self.gripper_goal
            self.gripper_publisher.publish(gripper_command)

        else:
            grasp_command = GraspActionGoal()
            grasp_command.goal.width = self.gripper_goal
            grasp_command.goal.force = 1
            grasp_command.goal.speed = 0.1
            grasp_command.goal.epsilon.inner = 0.05
            grasp_command.goal.epsilon.outer = 0.05
            self.active_grasp_id = grasp_command.goal_id
            self.grasp_publisher.publish(grasp_command)

    
if __name__ == '__main__':
    rospy.init_node('xbox_controller_publisher')
    rospy.loginfo('[Init] Xbox Controller Publisher')
    publisher = XboxControllerPublisher()
    rospy.spin()
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     publisher.publish_gripper_command()
    #     rate.sleep()
