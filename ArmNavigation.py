import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import hello_helpers.hello_misc as hm
from geometry_msgs.msg import PoseStamped
from lotion_application.srv import ArmMotion
from lotion_applicaiton.msg import ArmJointPos
import stretch_funmap.manipulation_planning as mp

class ArmNavigationNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        towel_loc_subscriber = self.create_subscription(PoseStamped,
                'detection/towel', callback=self.towel_loc_callback, qos_profile=10)
        target_loc_subscriber = self.create_subscription(PoseStamped,
                'detection/target', callback=self.target_loc_callback, qos_profile=10)
        self.towel_loc = None
        self.target_loc = None
        self.srv = self.create_service(ArmMotion, 'create_arm_path', self.serv_callback)

    def serv_callback(self, request, response):
        self.get_logger().info('ArmNavigationNode received request: ' + request.motion_type)
        if request.motion_type == 'towel':
            response.position_list = self.get_towel_path()
        elif request.motion_type == 'target':
            response.position_list = self.get_target_path()
        else:
            response.position_list = None
        return response

    def get_towel_path(self):
        '''

        :return:
        '''
        # get above target, lower arm to right above target, then close gripper
        pass

    def get_target_path(self):
        '''
        Determines an appropriate wiping pattern as a sequence of arm joint positions.
        :return:
        '''
        # get above target, lower arm to one corner, wipe in zig zag motion until whole surface
        # is covered
        pass

    def towel_loc_callback(self, pose_stamped):
        self.towel_loc = pose_stamped

    def target_loc_callback(self, pose_stamped):
        self.target_loc = pose_stamped

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
