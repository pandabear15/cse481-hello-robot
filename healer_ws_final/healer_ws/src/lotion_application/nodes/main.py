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
from lotion_application_msgs.action import CallRobot

from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer
from std_srvs.srv import Trigger

from lotion_application.InverseKinematics import InverseKinematics
from lotion_application.FrameListener import FrameListener
from lotion_application.MovementExecutor import MovementExecutor
from lotion_application.Trajectories import Trajectories
from lotion_application.TargetPoint import TargetPoint

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

        self.movementExecutor = MovementExecutor(self)
        self.trajectories = Trajectories()
        self.point = TargetPoint()
        self.ik = InverseKinematics(self)

        # create an action server so web app can trigger execution
        self.action_server = ActionServer(self, CallRobot, 'lotion_application', self.execute_application)
        
        # we shouldn't need a new thread once action server is implemented
        # dispatch a new thread to execute application, so that the main thread can continue it's job above
        self.execute_thread = threading.Thread(target=self.execute_application)
        self.execute_thread.start()
    
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
        #print(self.joint_state.name)
        #print(self.joint_state.position)
        #time.sleep(3.0)

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
    
    # execute all specified trajectories back to back. trajectories is a 2d array. the rows are trajectories, and columns
    # are trajectory elements. each trajectory has 4 parts. The first 3 parts correspond to the MovementExecutor.execute  
    # function's parameters (joint names, seconds, positions). The 4th part is status info to identify a finished trajectory
    def executeTrajectories(self, trajectories):
        for trajectory in trajectories:
            self.movementExecutor.execute(trajectory[0], False, trajectory[1], trajectory[2])
            success = self.wasTrajectorySuccessful2(trajectory[3])
            if (not success): return False

        return True

    def execute_application(self, goal_type):

        goal = goal_type.action_type
        
        # don't proceed unless joint state data has been received from ROS
        while (True):
            if (self.joint_state != None):
                break

        #old cloth test code
        while (True):
            self.executeTrajectories(self.trajectories.CleanForearm())
            user_input = input('Terminate?\n')
            valid_confirmation_inputs = ['yes', 'Yes', 'end', 'End', 'Y', 'y']
            if (user_input in valid_confirmation_inputs):
                break

        return
    
        #change this to getting towel position - move to towel
        self.ik_move(self.point.GetTestPoint())

        print("closing gripper")
        self.executeTrajectories(self.trajectories.close_gripper())

        print("move to person")
        self.ik_move(self.point.GetTestPoint2())

        # test points
        wrist_loc = [0, -0.8, 0.8]
        elbow_loc = [0, -0.4, 0.8]
        left_shoulder = [0.17, -1.1, 1.1]
        right_shoulder = [-0.17, -1.1, 1.1]
        left_elbow = [0.17, -1.0, 0.7]
        right_elbow = [-0.17, -1.0, 0.7]

        # in the future, add control flow between forearm and back
        if goal == 'forearm':
            print("calibrating forearm points")
            num_points = self.trajectories.calibrate_num_forearm_points(wrist_loc, elbow_loc)

            for i in range(num_points):
                next_point = self.trajectories.get_forearm_point(wrist_loc, elbow_loc, i)
                print("moving to forearm point " + str(i) + " at position " + str(next_point))
                self.ik_move(next_point)
                #self.executeTrajectories(self.trajectories.back_clean_motion())
        elif goal == 'back':
            print("calibrating back points")
            num_points = self.trajectories.calibrate_num_back_points()

            for i in range(num_points):
                next_point = self.trajectories.get_back_point(wrist_loc, elbow_loc, i)
                print("moving to back point " + str(i) + " at position " + str(next_point))
                self.ik_move(next_point)
                #self.executeTrajectories(self.trajectories.back_clean_motion())           

        # rclpy.shutdown()
    
            # user_input = input('Terminate?\n')
            # valid_confirmation_inputs = ['yes', 'Yes', 'end', 'End', 'Y', 'y']
            # if (user_input in valid_confirmation_inputs):
            #     break

        return_msg = CallRobot.Result()
        return_msg.completion = 0

        return return_msg
        
    def ik_move(self, target_point):
        ik_success = False
        executeTranslationOnce = True # for testing, executes robot base translation AT MOST once (even if target point isnt updated)

        while (not ik_success):
            # if target point is not within arm's reach, translate + rotate the robot base towards it
            if (not self.point.WithinReach(target_point) and executeTranslationOnce):
                self.executeTrajectories(self.trajectories.GoToTargetPoint(target_point, True))
                target_point = self.point.GetTestPoint()
                executeTranslationOnce = False
                continue
                
            # if target point is withina arm's reach, execute IK 
            (success, ik_trajectories) = self.ik.execute(target_point)
            if (success):
                print("executing ik trajectory")
                ik_success = self.executeTrajectories(ik_trajectories)
            else:
                print("failed to receieve ik trajectories")

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

