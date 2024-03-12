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

# ROS notes:
# negative x value means right (relative to arm)
# negative y value means forward (relative to arm)
# z value means verticalness (0 moves arm to ground, 1.1 moves arm to the top)
class TargetPoint():
    
    # point forward and to the right
    def GetTestPoint(self):
        return [-1.0, -1.0, 0.5]
    
    # point backward and to the left
    def GetTestPoint2(self):
        return [4.0, 4.0, 1.1]
    
    # returns whether target point is within arm's reach (no need for base rotation or translation)
    def WithinReach(self, target_point):
        x = target_point[0]
        y = target_point[1]
        z = target_point[2]

        # point is too low or too high
        if (z < 0.0  or z > 1.1):
            print("Error. vertical z value: {" + z + "} should be in range [0,1.1]")
            return False
        
        # point isn't too far offset to the left or right of the arm
        pointIsAlignedWithArm = math.fabs(x) < 0.2

        # point is in front of arm, and within wrist extension's reach
        pointIsInFrontOfArm = -1.1 < y and y < -0.5

        return pointIsAlignedWithArm and pointIsInFrontOfArm

