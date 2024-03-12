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

from lotion_application.InverseKinematics import InverseKinematics
from lotion_application.FrameListener import FrameListener
from lotion_application.MovementExecutor import MovementExecutor

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
                    [0.6, 0.39, 0.0, -3.14, 3.14]] # point gripper down
        
        finishedStatus1 = {'min_duration': 4.0, 'end_limb': "joint_gripper_finger_right", 
                        'end_pos': 0.22, 'epsilon_offset': 0.02}
                    
        # trajectory 2
        joint_names2 = ['joint_lift', 'wrist_extension', 'joint_wrist_roll', 'joint_wrist_pitch',
                    'joint_gripper_finger_right']
        seconds2 = [0.0, 2.0] 
        positions2 = [[0.55, 0.39, 0.0, -3.14, -2.9], # close gripper on towel
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
    

    def CleanBack(self):
        # likely will be significant amount of translation needed both x and y
        pass
    

    def complex_clean_motion(self):
        '''
        Not great in practice, but great for testing.
        '''
    
        start_y = random.random() * -1.0
        joint_names = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        seconds = [0.0, 0.2, 0.4, 0.6] 
        positions = [[0.1, 2.0, -2.0],
                      [0.0, -2.0, 2.0],
                      [-0.1, 2.0, -2.0],
                      [0.0, -2.0, 2.0]] 
        finishedStatus = {'min_duration': 8.0, 'end_limb': "joint_wrist_yaw", 
                        'end_pos': 0, 'epsilon_offset': 1.0}

        return [[joint_names, seconds, positions, finishedStatus]]
    
    def forearm_clean_motion(self):
        '''
        Manual motion that does a wrist roll to emulate a cleaning motion.
        '''
        joint_names = ['joint_wrist_roll']
        seconds = [0.0, 0.2, 0.4, 0.6] 
        positions = [[-2.0],
                      [2.0],
                      [-2.0],
                      [2.0]] 
        finishedStatus = {'min_duration': 8.0, 'end_limb': "joint_wrist_yaw", 
                        'end_pos': 0, 'epsilon_offset': 1.0}
        return [[joint_names, seconds, positions, finishedStatus]]

    def close_gripper(self):
        joint_names = ['joint_gripper_finger_right']
        seconds = [0.0, 3.0] 
        positions = [[3.14], [-3.14]] 
        finishedStatus = {'min_duration': 6.0, 'end_limb': 'joint_gripper_finger_right', 
                        'end_pos': -0.46, 'epsilon_offset': 0.03}
        return [[joint_names, seconds, positions, finishedStatus]]
    

    def calibrate_num_forearm_points(self, wrist_loc, elbow_loc):
        calibration_len = 0.1
        vector_x = elbow_loc[0] - wrist_loc[0]
        vector_y = elbow_loc[1] - wrist_loc[1]
        vector_z = elbow_loc[2] - wrist_loc[2]
        distance = (vector_x * vector_x + vector_y * vector_y + vector_z * vector_z) ** 0.5
        self.num_steps_forearm = math.ceil(distance / calibration_len) + 1
        return self.num_steps_forearm


    def get_forearm_point(self, wrist_loc, elbow_loc, point_num):
        # numpy maybe? idk if this will cause casting issues
        # points should instruct the robot to move some number of times in one direction to move from wrist to elbow
        # should insert a cleaning motion in between every one
        # acceptable values for point_num range from 0 to self.num_steps_forearm inclusive
        assert 0 <= point_num and point_num <= self.num_steps_forearm
        vector_x = elbow_loc[0] - wrist_loc[0]
        vector_y = elbow_loc[1] - wrist_loc[1]
        vector_z = elbow_loc[2] - wrist_loc[2]
        scale = point_num / (self.num_steps_forearm - 1)
        return [elbow_loc[0] + vector_x * scale, elbow_loc[1] + vector_y * scale, elbow_loc[2] + vector_z * scale]
    
    
    def calibrate_num_back_points(self, left_shoulder, right_shoulder, left_elbow, right_elbow):
        calibration_len = 0.1
        # heck it np time
        left_shoulder = np.array(left_shoulder)
        right_shoulder = np.array(right_shoulder)
        left_elbow = np.array(left_elbow)
        right_elbow = np.array(right_elbow)
        width = np.linalg.norm(right_shoulder - left_shoulder)
        # little inexact but we figure out adjustments later
        height = np.linalg.norm(left_elbow - left_shoulder)
        self.num_steps_back_width = math.ceil(width / calibration_len) + 1
        self.num_steps_back_height = math.ceil(height / calibration_len) + 1
        return self.num_steps_back_width * self.num_steps_back_height
    

    def get_back_point(self, left_shoulder, right_shoulder, left_elbow, right_elbow, point_num):
        assert 0 <= point_num and point_num <= self.num_steps_back_width * self.num_steps_back_height
        point_num_width = point_num % self.num_steps_back_width
        point_num_height = int(point_num / self.num_steps_back_width)
        left_shoulder = np.array(left_shoulder)
        right_shoulder = np.array(right_shoulder)
        left_elbow = np.array(left_elbow)
        right_elbow = np.array(right_elbow)
        # if point_num_height is even, move left -> right, otherwise right -> left
        if point_num_height % 2 == 0:
            return (left_shoulder + (right_shoulder - left_shoulder) * (point_num_width / (self.num_steps_back_width - 1)) 
                    + (left_elbow - left_shoulder) * (point_num_height / (self.num_steps_back_height - 1)))
        else:
             return (left_shoulder + (right_shoulder - left_shoulder) * ((self.num_steps_back_width - 1 - point_num_width) / (self.num_steps_back_width - 1)) 
                    + (left_elbow - left_shoulder) * (point_num_height / (self.num_steps_back_height - 1)))           
    
    
    def TestOneLimb(self):
        
        # trajectory 0
        joint_names0 = ['wrist_extension', 'joint_wrist_pitch']
        seconds0 = [0.0, 2.0] 
        positions0 = [[0.4, 1.57],[0.4, 1.00]] # rotate it back if doing trajectory 3 + random starting positoin
        finishedStatus0 = {'min_duration': 10.0, 'end_limb': "joint_lift", 
                        'end_pos': 0, 'epsilon_offset': 2}

        # export trajectory info
        trajectories = [[]]
        trajectories[0] = [joint_names0, seconds0, positions0, finishedStatus0]

        return trajectories
    
    
    def GoToTargetPoint(self, target_point, stayFarFromPoint):

        x = target_point[0]
        y = target_point[1]

        y_translation = (-1.1 if stayFarFromPoint else -0.8) - y
        y_translation_sleep_length = math.fabs(y_translation * 3)

        # trajectory 1
        joint_names0= ['translate_mobile_base']
        seconds0 = [0.0] 
        positions0 = [[x]]
        finishedStatus0 = {'min_duration': y_translation_sleep_length, 'end_limb': "joint_lift", 
                        'end_pos': 0, 'epsilon_offset': 2}
        
        # don't add rotation trajectories if unneccessary (1 translation is much quicker)
        if (-1.1 < y and y < -0.5):
            trajectories = [[]]
            trajectories[0] = [joint_names0, seconds0, positions0, finishedStatus0]
            return trajectories

        # trajectory 2
        joint_names1 = ['rotate_mobile_base']
        seconds1 = [0.0]
        positions1 = [[-1.57]]
        finishedStatus1 = {'min_duration': 4.0, 'end_limb': "joint_lift", 
                        'end_pos': 0, 'epsilon_offset': 2}
        
        # trajectory 3
        joint_names2 = ['translate_mobile_base']
        seconds2 = [0.0] 
        positions2 = [[y_translation]]
        finishedStatus2 = {'min_duration': y_translation_sleep_length, 'end_limb': "joint_lift", 
                        'end_pos': 0, 'epsilon_offset': 2}

        # trajectory 4
        joint_names3 = ['rotate_mobile_base']
        seconds3 = [0.0]
        positions3 = [[1.57]]
        finishedStatus3 = {'min_duration': 4.0, 'end_limb': "joint_lift", 
                        'end_pos': 0, 'epsilon_offset': 2}
        
        # export trajectory info
        trajectories = [[],[],[],[]]
        trajectories[0] = [joint_names0, seconds0, positions0, finishedStatus0]
        trajectories[1] = [joint_names1, seconds1, positions1, finishedStatus1]
        trajectories[2] = [joint_names2, seconds2, positions2, finishedStatus2]
        trajectories[3] = [joint_names3, seconds3, positions3, finishedStatus3]

        # Axis of actual robot when using IK:
        # -x means to the right of arm
        # +x means to the left of arm
        # -y means in front of arm
        # +y means behind arm
        # negative translation means to the right of arm (-x -> 0)
        # positive translation means to the left of arm (+x -> 0)
        # -1.57 rotation means 90 degree clockwise rotation relative to arm
        # 1.57 rotation means 90 degree cc rotation relative to arm

        # -10, -10
        # 10, -10

        # if y is negative
        #   add 0.8
        # if y is positive
        #   subtract 0.8
        # then z = -y
   #
        # to move forwards
        # -1.57
        # +x the amount u want to travel forwards (by amount z)
        # +1.57

        # goal: 0, -0.8
        return trajectories