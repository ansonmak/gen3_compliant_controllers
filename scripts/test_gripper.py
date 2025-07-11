#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def control_gripper(position):
    # Create a JointTrajectory message
    traj = JointTrajectory()
    traj.joint_names = ['finger_joint']  # Replace with your gripper joint name(s)

    point = JointTrajectoryPoint()
    point.positions = [position]           # Desired gripper position
    point.velocities = [0.0]               # Optional
    point.time_from_start = rospy.Duration(1.0)  # 1 second to reach position

    traj.points.append(point)

    # Publish the trajectory command
    rospy.loginfo(f"Sending gripper command: position={position}")
    pub.publish(traj)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('gripper_commander', anonymous=True)

        # Publisher to the gripper controller command topic
        pub = rospy.Publisher('/gripper_position_controller/command', JointTrajectory, queue_size=10)

        # Wait for the publisher to be ready
        rospy.sleep(1)

        # Example: 0.8 = closed, 0.0 = open
        while not rospy.is_shutdown():
            control_gripper(float(input("Enter gripper position: ")))
            rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        pass