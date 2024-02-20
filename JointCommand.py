#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time

class JointCommand(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'joint_command', 'joint_command', wait_for_first_pointcloud=False)

    # execute movement of robot joints
    # frame_id: [string] frame used
    # joint_names: [array of strings] names of joints being moved
    # use_init_pos: [boolean] whether joints should preserve their current position at the start
    # second: [array of floats] total seconds elapsed till the next pose update
    # positions: [2d array of floats] positions for joints on each pose update. next row = next update, mth column = mth joint position
    def execute(self, frame_id, joint_names, use_init_pos, seconds, positions): 
        # wait till robot info is available
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue

        # throw errors if invalid parameters were passed in
        if (not use_init_pos and len(positions) != len(seconds)):
            print("error, positions array and seconds array have different lengths")
            return
        
        elif (use_init_pos and (len(positions) + 1) != len(seconds)):
            print("error. if using initial positon of joints, positions array should have 1 less row than the number of elements in seconds array.")
            return
        
        # create new stow actions
        self.get_logger().info('Stowing...')

        stow_points = []
        for i in range(len(seconds)):
            stow_points.append(JointTrajectoryPoint())

        for i in range(len(seconds)):
            stow_points[i].time_from_start = Duration(seconds[i].to_msg())

        # track pose update stage
        update_idx = 0

        if use_init_pos:
            start_positions = []
            for joint_idx in range(len(joint_names)):
                start_positions.append(self.getCurrentPos(joint_names[joint_idx]))

            stow_points[update_idx].positions = start_positions
            update_idx = 1
        
        for i in range(len(positions)):
            stow_points[update_idx] = positions[i]
            update_idx = update_idx + 1

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = stow_points
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = frame_id

        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")

    # execute specific robot movement testcase with just 2 position updates, from ROS 2 docs
    def execute_testcase(self):
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue

        self.get_logger().info('Stowing...')

        stow_point1 = JointTrajectoryPoint()
        stow_point2 = JointTrajectoryPoint()
        stow_point1.time_from_start = Duration(seconds=0.0).to_msg()
        stow_point2.time_from_start = Duration(seconds=4.0).to_msg()

        stow_point1.positions = [self.getCurrentPos('joint_lift'), self.getCurrentPos('wrist_extension'), self.getCurrentPos('joint_wrist_yaw')]
        stow_point2.positions = [0.2, 0.0, 3.14]

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [stow_point1, stow_point2]
        self.trajectory_client.send_goal_async(trajectory_goal)
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.get_logger().info("Goal sent")
    
    # returns current position [float] of specified joint_name [string]
    def getCurrentPos(self, joint_name):
        joint_idx = self.joint_state.name.index(joint_name)
        return self.joint_state.position[joint_idx]