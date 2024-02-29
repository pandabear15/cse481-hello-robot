import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import hello_helpers.hello_misc as hm

class ArmNavigationNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        # potentially more initialization steps needed here



    def frame_convert(self, old_pose, new_frame):
        """
        Converts a geometry_msgs/msg/PoseStamped message to a new PoseStamped message,
        in the frame specified by new_frame.
        :param old_pose: the PoseStamped message to convert
        :param new_frame: the desired frame to convert into
        :return: a new PoseStamped message representing the same pose reframed into the new frame.
        """
        pass

    def inverse_kinematics(self, pose):
        """
        Determines the position of the robot joints necessary to get the gripper to achieve a particular
        pose.
        :param pose: the pose the gripper wants to achieve.
        :return: a dictionary containing joints and the desired position.
        The dictionary keys should be in ['wrist_extension', 'joint_lift', 'joint_head_pan',
                'joint_head_tilt', 'joint_wrist_yaw', 'jlint_wrist_pitch', 'joint_wrist_roll',
                'joint_gripper_finger_left'].
        """
        pass