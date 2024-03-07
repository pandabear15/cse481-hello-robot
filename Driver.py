import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import hello_helpers.hello_misc as hm
import JointCommand


class MotionNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.arm_client = self.create_client()

    def main(self):
        # keep track of internal state: call arm navigation, grab towel, close gripper, stow,
        # call arm navigation, do rubbing motion, stow.
        pass

