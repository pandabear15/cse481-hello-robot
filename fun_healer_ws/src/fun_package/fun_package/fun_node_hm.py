#!/usr/bin/env python3

import hello_helpers.hello_misc as hm
# from rclpy.node import Node
import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient


class FunNode(hm.HelloNode):
# class FunNode(Node):
    def __init__(self):
        print('we initing bois')
        hm.HelloNode.__init__(self)
        print('maining')
        hm.HelloNode.main(self, 'fun_node', 'fun_node', wait_for_first_pointcloud=False)
        
        self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.fake_rviz = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.init_point = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.local_local_cli = self.create_client(Trigger, '/funmap/trigger_local_localization')
        self.reach_until_contact_cli = self.create_client(Trigger, '/funmap/trigger_reach_until_contact')
        #self.some_action_cli = ActionClient(self, SomeAction, '/some_action')

        # self.open_gripper()
        # self.close_gripper()

        # #approx point: 1.731, 0.3462, 0.7167
        self.two_d_pose_estimate(self.pose_with_covariance_stamped('map', 1.731, 0.3462, 0.0, 0.0, 0.0, 0.0, 1.0))
        self.local_localization()

        

        # just some interesting points
        #goal = self.point_stamped('map', 0.182176, -0.508267, 0.90669)
        #goal = self.point_stamped('map', -1.50507, 2.40945, 0.8)
        # goal = self.point_stamped('map', 2.667, -0.1, 0.5)

        self.put_gripper_at_point('map', 2.667, -0.1, 0.5)

        # lol this is at end ig (???)
        self.new_thread.join()



    def reach_until_contact(self):
        while not self.reach_until_contact_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reach_until_contact_req = Trigger.Request()
        self.get_logger().info('calling reach until contact service')
        result = self.reach_until_contact_cli.call(self.reach_until_contact_req)
        self.get_logger().info(f'result of reach until contact: {result}')

    def put_gripper_at_point(self, frame_id, x, y, z):
        goal = self.point_stamped(frame_id, x, y, z)
        
        self.get_logger().info(f'publishing the following point: {goal}')
        self.fake_rviz.publish(goal)

        # todo: wait until completed. probably some action BS?

    # init_pose is a PoseWithCovarianceStamped
    def two_d_pose_estimate(self, init_pose):
        self.get_logger().info(f'publishing initial pose: {init_pose}')
        self.init_point.publish(init_pose)

    def local_localization(self):
        while not self.local_local_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.local_local_req = Trigger.Request()
        self.get_logger().info('calling local localization service')
        result = self.local_local_cli.call(self.local_local_req)
        self.get_logger().info(f'result of local localization: {result}')

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
        #actually idk if i need this lmao

    def pose_stamped(self, frame_id, x, y, z, w):
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        # ps.header.stamp = self.get_clock().now()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = w
        return ps
    
    def point_stamped(self, frame_id, x, y, z):
        ps = PointStamped()
        ps.header.frame_id = frame_id
        # ps.header.stamp = self.get_clock().now()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x = x
        ps.point.y = y
        ps.point.z = z
        return ps
    
    def pose_with_covariance_stamped(self, frame_id, x, y, z, qx, qy, qz, qw):
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.frame_id = frame_id
        pwcs.header.stamp = self.get_clock().now().to_msg()
        # pwcs.pose.covariance = cov #idk what covariance is
        pwcs.pose.pose.position.x = x
        pwcs.pose.pose.position.y = y
        pwcs.pose.pose.position.z = z
        pwcs.pose.pose.orientation.x = qx
        pwcs.pose.pose.orientation.y = qy
        pwcs.pose.pose.orientation.z = qz
        pwcs.pose.pose.orientation.w = qw
        return pwcs

    def open_gripper(self):
        self.get_logger().info('Opening gripper...')
        self.move_to_pose({'joint_gripper_finger_right': 2.0})
        self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        self.get_logger().info('Closing gripper...')
        self.move_to_pose({'joint_gripper_finger_right': -2.9})
        self.get_logger().info('Gripper closed')
        

def main(args=None):
    try:
        node = FunNode()
        #node.main()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
    


if __name__ == '__main__':
    main()
