#!/usr/bin/env python3

from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from rclpy.node import Node
from rclpy.action import ActionClient
import time

class MovementExecutor(Node):
    def __init__(self, Robot):
        super().__init__('read_write_api')
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        self.Robot = Robot
        
    # execute movement of robot joints using FollowJointTrajectory
    # joint_names: [array of strings] names of joints being moved
    # use_init_pos: [boolean] whether joints should preserve their current position at the start
    # seconds: [array of floats] index i -> seconds elapsed from start before transitioning to the ith pose update
    # positions: [2d array of floats] row i, column j -> specifies jth joint's position on ith pose update
        
    def execute(self, joint_names, use_init_pos, seconds, positions): 
        # wait till robot info is available
        while self.Robot.joint_state == None:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue

        # throw errors if invalid parameters were passed in
        if (not use_init_pos and len(positions) != len(seconds)):
            print("error, positions array and seconds array have different lengths")
            return
        
        elif (use_init_pos and ((len(positions) + 1) != len(seconds) or seconds[0] != 0)):
            print("error. since we're using initial positon of joints at time = 0, positions array should have 1 less row than the number of " +
                  "elements in seconds array. Also set the first element of the seconds array to 0")
            return 
        
        # create new stow actions
        self.get_logger().info('Stowing...')

        stow_points = []
        for i in range(len(seconds)):
            stow_points.append(JointTrajectoryPoint())

        for i in range(len(seconds)):
            stow_points[i].time_from_start = Duration(seconds=seconds[i]).to_msg()

        # track pose update stage
        update_idx = 0

        if use_init_pos:
            start_positions = []
            for joint_idx in range(len(joint_names)):
                start_positions.append(self.Robot.readCurrentPos(joint_names[joint_idx]))

            stow_points[update_idx].positions = start_positions
            update_idx = 1
        
        for i in range(len(positions)):
            stow_points[update_idx].positions = positions[i]
            update_idx = update_idx + 1

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = joint_names

        for i in range(len(stow_points)):
            print(stow_points[i])
        
        trajectory_goal.trajectory.points = stow_points
        self.trajectory_client.send_goal_async(trajectory_goal)
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'