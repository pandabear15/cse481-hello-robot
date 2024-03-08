#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import hello_helpers.hello_misc as hm
from hello_helpers.hello_misc import HelloNode
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from InverseKinematics import InverseKinematics
from FrameListener import FrameListener
from MovementExecutor import MovementExecutor

import math
import random
import time
import numpy as np

class Trajectories():
    
    def GrabTowel(self):
        
        # trajectory 0
        joint_names0 = ['rotate_mobile_base','joint_lift', 'wrist_extension', 'joint_wrist_roll', 'joint_wrist_pitch',
                    'joint_gripper_finger_right']
        seconds0 = [0.0] 
        positions0 = [[-1.4, 0.9, 0.3, -2.8, -2.8, 2.0]] # rotate it back if doing trajectory 3 + random starting positoin
        finishedStatus0 = {'min_duration': 3.0, 'end_limb': "joint_lift", 
                        'end_pos': 0, 'epsilon_offset': 2}

        # trajectory 1
        joint_names1 = ['joint_lift', 'wrist_extension', 'joint_wrist_roll', 'joint_wrist_pitch',
                    'joint_gripper_finger_right']
        seconds1 = [0.0, 2.0] 
        positions1 = [[0.8, 0.39, 0.0, 0.0, 2.0],  # extend arm forward and get gripper oriented nicely
                    [0.8, 0.39, 0.0, -3.14, 3.14]] # point gripper down
        
        finishedStatus1 = {'min_duration': 4.0, 'end_limb': "joint_gripper_finger_right", 
                        'end_pos': 0.22, 'epsilon_offset': 0.02}
                    
        # trajectory 2
        joint_names2 = ['joint_lift', 'wrist_extension', 'joint_wrist_roll', 'joint_wrist_pitch',
                    'joint_gripper_finger_right']
        seconds2 = [0.0, 2.0] 
        positions2 = [[0.7, 0.39, 0.0, -3.14, -2.9], # close gripper on towel
                      [0.8, 0.25, 0.0, 0.0, -2.9]] # lift arm back up
        finishedStatus2 = {'min_duration': 5.5, 'end_limb': "joint_lift", 
                        'end_pos': 0.8, 'epsilon_offset': 0.02}
        
        # trajectory 3
        joint_names3 = ['rotate_mobile_base', 'joint_wrist_roll', 'joint_wrist_pitch']
        seconds3 = [0.0, 4.0] 
        positions3 = [[1.4, 0.36, 0.0], # rotate ~90 degrees counter clockwise
                      [1.0, 0.36, 0.0],] # rotate a little more counter clockwise
        finishedStatus3 = {'min_duration': 9.0, 'end_limb': "joint_lift", 
                        'end_pos': 0.8, 'epsilon_offset': 0.02}
        
        # export trajectory info
        trajectories = [[],[],[],[]]
        trajectories[0] = [joint_names0, seconds0, positions0, finishedStatus0]
        trajectories[1] = [joint_names1, seconds1, positions1, finishedStatus1]
        trajectories[2] = [joint_names2, seconds2, positions2, finishedStatus2]
        trajectories[3] = [joint_names3, seconds3, positions3, finishedStatus3]

        return trajectories
    
    
    def CleanForearm(self):
        
        #period = 2
        #y = start_y + math.sin(time.time_ns * 2 * math.pi / period)
        #x = math.cos(time.time_ns * 2 * math.pi / period)
        # print(time)


        # trajectory 0
        start_y = random.random() * -0.5  # gripper points downwards lightly
        joint_names0 = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        seconds0 = [0.0, 0.2, 0.4, 0.6] 
        positions0 = [[0.1, start_y, -2.0],
                      [0.0, start_y + 0.1, 2.0],
                      [-0.1, start_y, -2.0, -2.0],
                      [0.0, start_y - 0.1, 2.0]] 
        finishedStatus0 = {'min_duration': 10.0, 'end_limb': "joint_wrist_yaw", 
                        'end_pos': 0, 'epsilon_offset': 1.0}
        
        
        # trajectory 1
        start_y = random.random() * -1.0
        joint_names1 = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        seconds1 = [0.0, 0.2, 0.4, 0.6] 
        positions1 = [[0.1, start_y, -2.0],
                      [0.0, start_y + 0.1, 2.0],
                      [-0.1, start_y, -2.0, -2.0],
                      [0.0, start_y - 0.1, 2.0]] 
        finishedStatus1 = {'min_duration': 10.0, 'end_limb': "joint_wrist_yaw", 
                        'end_pos': 0, 'epsilon_offset': 1.0}

                        
        # trajectory 2
        start_y = random.random() * -1.0
        joint_names2 = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        seconds2 = [0.0, 0.2, 0.4, 0.6] 
        positions2 = [[0.1, start_y, -2.0],
                      [0.0, start_y + 0.1, 2.0],
                      [-0.1, start_y, -2.0, -2.0],
                      [0.0, start_y - 0.1, 2.0]] 
        finishedStatus2 = {'min_duration': 10.0, 'end_limb': "joint_wrist_yaw", 
                        'end_pos': 0, 'epsilon_offset': 1.0}
        
        # trajectory 3
        start_y = random.random() * -1.0
        joint_names3 = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        seconds3 = [0.0, 0.2, 0.4, 0.6] 
        positions3 = [[0.1, start_y, -2.0],
                      [0.0, start_y + 0.1, 2.0],
                      [-0.1, start_y, -2.0, -2.0],
                      [0.0, start_y - 0.1, 2.0]] 
        finishedStatus3 = {'min_duration': 10.0, 'end_limb': "joint_wrist_yaw", 
                        'end_pos': 0, 'epsilon_offset': 1.0}

        # export trajectory info
        trajectories = [[],[],[],[]]
        trajectories[0] = [joint_names0, seconds0, positions0, finishedStatus0]
        trajectories[1] = [joint_names1, seconds1, positions1, finishedStatus1]
        trajectories[2] = [joint_names2, seconds2, positions2, finishedStatus2]
        trajectories[3] = [joint_names3, seconds3, positions3, finishedStatus3]

        return trajectories