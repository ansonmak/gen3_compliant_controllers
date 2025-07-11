#!/usr/bin/env python
import sys

import pygame
import numpy as np
import rospy
import tf2_ros
import tf_conversions
from controller_manager_msgs.srv import SwitchController
from kortex_hardware.srv import ModeService
from moveit_msgs.msg import CartesianTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamic_reconfigure.client import Client

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected. Make sure the PS4 controller is connected.")
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Detected controller: {joystick.get_name()}")

def get_delta_quat(r, p ,y):
    return tf_conversions.transformations.quaternion_from_euler(r, p, y)

def get_quat_mult(q1, q2):
    return tf_conversions.transformations.quaternion_multiply(q1, q2)


if __name__ == "__main__":
    rospy.init_node('teleop_task_controller_py')
    mode = "effort"
    topic = "/task_space_compliant_controller/command"
        
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("Switching to", "task", "space controller. The robot might jerk a bit as it is switching to effort mode.")
    rospy.wait_for_service("controller_manager/switch_controller")
    rospy.wait_for_service("set_control_mode")
    switch_controller = rospy.ServiceProxy(
        "controller_manager/switch_controller", SwitchController
    )
    mode_change = rospy.ServiceProxy("set_control_mode", ModeService)

    try:
        resp = switch_controller(
            ["task_space_compliant_controller"],
            ["velocity_controller", "joint_space_compliant_controller"],
            1,
            0,
            5,
        )
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    try:
        resp1 = mode_change(mode)
        print("Mode change response: ", resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    print("Switch complete. You can now send commands to the robot.")

    try:
        cmd_pub = rospy.Publisher(topic, CartesianTrajectoryPoint, queue_size=10)
        rate = rospy.Rate(60.0)
        init_pose = tfBuffer.lookup_transform('base_link', 'end_effector_link', rospy.Time())

        init_quat = [init_pose.transform.rotation.x, init_pose.transform.rotation.y, init_pose.transform.rotation.z, init_pose.transform.rotation.w]

        while not rospy.is_shutdown():
            pygame.event.pump()
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            x_input = joystick.get_axis(4)
            y_input = joystick.get_axis(3)
            z_up_button = joystick.get_button(7) 
            z_up = joystick.get_axis(5) + 1.0
            z_down_button = joystick.get_button(6) 
            z_down = joystick.get_axis(2) + 1.0

            roll_input = joystick.get_hat(0)[0] 
            pitch_input = joystick.get_axis(1)
            yaw_input = -joystick.get_axis(0)


            send_cmd = False

            if (abs(x_input) > 0.1):
                init_pose.transform.translation.x -= x_input * 0.005
                send_cmd = True

            if (abs(y_input) > 0.1):
                init_pose.transform.translation.y -= y_input * 0.005
                send_cmd = True    

            if (z_up_button):
                init_pose.transform.translation.z += z_up * 0.0025
                send_cmd = True 
                
            if (z_down_button):
                init_pose.transform.translation.z -= z_down * 0.0025
                send_cmd = True 

            if (abs(roll_input) > 0):
                d_quat = get_delta_quat(roll_input * 0.01, 0, 0)
                init_quat = get_quat_mult(d_quat, init_quat)
                send_cmd = True 

            if (abs(pitch_input) > 0):
                d_quat = get_delta_quat(0, pitch_input * 0.01, 0)
                init_quat = get_quat_mult(d_quat, init_quat)
                send_cmd = True 

            if (abs(yaw_input) > 0):
                d_quat = get_delta_quat(0, 0, yaw_input * 0.01)
                init_quat = get_quat_mult(d_quat, init_quat)
                send_cmd = True 

            if send_cmd:
                print(init_pose.transform)
                cmd = CartesianTrajectoryPoint()
                cmd.point.pose.position.x = init_pose.transform.translation.x
                cmd.point.pose.position.y = init_pose.transform.translation.y
                cmd.point.pose.position.z = init_pose.transform.translation.z
                cmd.point.pose.orientation.x = init_quat[0]
                cmd.point.pose.orientation.y = init_quat[1]
                cmd.point.pose.orientation.z = init_quat[2]
                cmd.point.pose.orientation.w = init_quat[3]
                cmd_pub.publish(cmd)
                send_cmd = False

            rate.sleep()
    
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)