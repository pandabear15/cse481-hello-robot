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
from Trajectories import Trajectories

import time
import math
import numpy as np
import threading

class Robot(Node):
    def __init__(self):

        # required initialization
        super().__init__('begin_application')
        self.joint_state = None

        # main thread should listen for joint_state updates (ex. current positions of limbs)
        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        
        # dispatch a new thread to execute application, so that the main thread can continue it's job above
        self.execute_thread = threading.Thread(target=self.execute_application)
        self.execute_thread.start()
    
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
        # print(self.joint_state)
        # time.sleep(0.5)

    # returns current position of a specific joint
    def readCurrentPos(self, joint_name):
        if (joint_name in self.joint_state.name):
            joint_idx = self.joint_state.name.index(joint_name)
            return self.joint_state.position[joint_idx]
        else:
            return 0.0

    # quickly check if a Follow Trajectory Goal has successfully finished.
    # takes in a joint name, end joint position, and returns whether the joint's current position 
    # is within epsilon of the end joint position
    def wasTrajectorySuccessful(self, joint_name, targetJointPos, epsilon):
        posDiff = self.readCurrentPos(joint_name) - targetJointPos
        return (math.fabs(posDiff) < epsilon)
    
    # repeatedly check if a Follow Trajectory Goal has successfully finished
    # do 30 checks, once every 0.1 seconds. only return false if all checks fail
    def wasTrajectorySuccessful2(self, status):
        time.sleep(status['min_duration'])

        checks = 30
        success = self.wasTrajectorySuccessful(status['end_limb'], status['end_pos'], status['epsilon_offset'])
        
        while (not success and checks > 0):
            time.sleep(0.1)
            success = self.wasTrajectorySuccessful(status['end_limb'], status['end_pos'], status['epsilon_offset'])
            checks = checks-1

        return success
    
    # execute multiple movement trajectories back to back. trajectories is a 2d array. the rows are trajectories, and columns
    # are trajectory elements. each trajectory has 4 parts. The first 3 parts correspond to the MovementExecutor.execute  
    # function's parameters (joint names, seconds, positions). The 4th part is status info to identify a finished trajectory
    def executeTrajectories(self, movementExecutor, trajectories):
        for trajectory in trajectories:
            movementExecutor.execute(trajectory[0], False, trajectory[1], trajectory[2])
            success = self.wasTrajectorySuccessful2(trajectory[3])
            if (not success): return

    def execute_application(self):
        
        # don't proceed unless joint state data has been received from ROS
        while (True):
            if (self.joint_state != None):
                break

        movementExecutor = MovementExecutor(self)
        trajectories = Trajectories()
        
        while (True):
            self.executeTrajectories(movementExecutor, trajectories.CleanForearm())
            user_input = input('Terminate?\n')
            valid_confirmation_inputs = ['yes', 'Yes', 'end', 'End', 'Y', 'y']
            if (user_input in valid_confirmation_inputs):
                break
        
        while (True):
            time.sleep(1)
            
        # TO-DO: Finish fixing Inverse Kinematics before uncommenting
            
        # ik = InverseKinematics(read_write_api, base.Base())
        # target_point = [-0.043, -0.441, 0.654]
        #    if True:
        #     # Calculate target point based on a sine wave
        #     t = time.time()  # Get current time
        #     offset = 0  # Offset for the sine wave
        #     amplitude = 2 # Amplitude of the sine wave
        #     period = 2
        #     x = 1 #* math.sin(2 * math.pi * frequency * t) + offset
        #     y = 10# Assuming y-coordinate remains constant
        #     z = 0.5#* math.sin(2 * math.pi / period * t) + offset# Assuming z-coordinate remains constant
        #     target_point = [x, y, z]
        #     print(target_point)
            
        #     ik.execute(target_point)
        #     # time.sleep(1.2)
        
        # time.sleep(100)
        # rclpy.shutdown()
    
    
        
def main(args=None):
    rclpy.init()
    
    # uncomment if not calibrated
    # read_write_api.home_the_robot()
    # read_write_api.switch_to_position_mode()

    begin_robot = Robot()
    try:
        rclpy.spin(begin_robot)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

