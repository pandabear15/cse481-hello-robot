#!/usr/bin/env python3

import hello_helpers.hello_misc as hm
from rclpy.node import Node
import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header


#class FunNode(hm.HelloNode):
class FunNode(Node):
    def __init__(self):
        print('we initing bois')
        super().__init__('fun_node')
        # hm.HelloNode.__init__(self)
        # print('maining')
        # hm.HelloNode.main(self, 'fun_node', 'fun_node', wait_for_first_pointcloud=False)
        
        
        print("ok start publishing")
        self.init_point = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        init_pose = self.pose_with_covariance_stamped('map', 4.49, 0.4, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.init_point.publish(init_pose)



        # goal = self.point_stamped('map', 0.182176, -0.508267, 0.90669)
        # #goal = self.point_stamped('map', -1.50507, 2.40945, 0.8)
        
        # self.fake_rviz = self.create_publisher(PointStamped, '/clicked_point', 10)
        
        # print('publishing . . . . .')
        # self.fake_rviz.publish(goal)
        # print('goal was published')
        # print(goal)
        

    def debug_subscriptions(self):
        self.sub_move_base_action_status = self.create_subscription()

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


    # def pose_stamped(self, header, pose):
    #     ret = PoseStamped()
    #     ret.header = header
    #     ret.pose = pose
    #     return ret

    def header(self, seq_id, time_stamp, framde_id):
        ret = Header()
        ret.stamp = self.get_clock().now()
        ret.frame_id = framde_id
    
        

def main(args=None):
    try:
        rclpy.init(args=args)
        node = FunNode()
        rclpy.spin(node)
        #node.main()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
    


if __name__ == '__main__':
    main()
