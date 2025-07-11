#!/usr/bin/env python
import rospy
from dynamic_reconfigure.client import Client

def main():
    rospy.init_node("reconfigure_test_client")
    client = Client("/kortex_hardware", timeout=5)

    # Set new parameters
    default_params = {
        "k_x": 200.0,
        "k_y": 200.0,
        "k_z": 200.0,
        "k_roll": 100.0,
        "k_pitch": 100.0,
        "k_yaw": 100.0,
        "d_x": 40.0,
        "d_y": 40.0,
        "d_z": 40.0,
        "d_roll": 20.0,
        "d_pitch": 20.0,
        "d_yaw": 20.0,
    }

    stiff_params = {
        "k_x": 250.0,
        "k_y": 250.0,
        "k_z": 250.0,
        "k_roll": 250.0,
        "k_pitch": 250.0,
        "k_yaw": 250.0,
        "d_x": 80.0,
        "d_y": 80.0,
        "d_z": 80.0,
        "d_roll": 60.0,
        "d_pitch": 60.0,
        "d_yaw": 60.0,
    }

    door_params = {
        "k_x": 250.0,
        "k_y": 200.0,
        "k_z": 250.0,
        "k_roll": 150.0,
        "k_pitch": 250.0,
        "k_yaw": 100.0,
        "d_x": 80.0,
        "d_y": 40.0,
        "d_z": 80.0,
        "d_roll": 30.0,
        "d_pitch": 60.0,
        "d_yaw": 30.0,
    }
    
    test_params = {
        "k_x": 250.0,
        "k_y": 250.0,
        "k_z": 250.0,
        "k_roll": 50.0,
        "k_pitch": 50.0,
        "k_yaw": 50.0,
        "d_x": 80.0,
        "d_y": 80.0,
        "d_z": 80.0,
        "d_roll": 10.0,
        "d_pitch": 10.0,
        "d_yaw": 10.0,
    }

    config = client.update_configuration(stiff_params)
    rospy.loginfo("Updated parameters: %s", config)

if __name__ == "__main__":
    main()