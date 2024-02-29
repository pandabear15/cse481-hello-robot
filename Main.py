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

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)
        frame_list = []  # unsure how to do this yet
        pose_list = {}
        # rclpy.init()
        while True:
            user_input = input("Awaiting input: \n")
            if user_input[0:2] == "-s":
                pose_name = user_input[2:].strip()
                # get position and save it as pose
                pose_list[pose_name] = self.get_joint_dict(self.joint_state)
                print(pose_list[pose_name])
                print("Saved current pose as " + pose_name)
            elif user_input[0:2] == "-v":
                pose_name = user_input[2:].strip()
                if pose_name == "":
                    print("List of available poses")
                    for pose in pose_list.keys():
                        print(pose)
                else:
                    print("figure something out here about frames")  # todo
            elif user_input[0:2] == "-c":
                commands = user_input[2:].strip().split(" ")
                print(commands)
                # verify all poses exist, then execute all of them
                if all(command in pose_list.keys() for command in commands):
                    for command in commands:
                        print("Executing " + command)
                        pose = pose_list[command]
                        self.move_to_pose(pose,blocking=True) #  may need some code to wait for this to complete?
                        print('Completed pose ' + command)
                else:
                    print("Unknown list of commands.")
    
    def get_joint_dict(self, msg):
        important_joints = ['wrist_extension', 'joint_lift', 'joint_head_pan', 'joint_head_tilt', 'joint_wrist_yaw', 'jlint_wrist_pitch', 'joint_wrist_roll',
                            'joint_gripper_finger_left']
        pos_dict = {}
        for i in range(len(msg.name)):
            if msg.name[i] in important_joints:
                pos_dict[msg.name[i]] = msg.position[i]
        return pos_dict
    

# might need to reference this code later

# class FrameListener(Node):
#
#     def __init__(self):
#         super().__init__('stretch_tf_listener')
#
#         self.declare_parameter('target_frame', 'link_grasp_center')
#         self.target_frame = self.get_parameter(
#             'target_frame').get_parameter_value().string_value
#
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#
#         time_period = 1.0 # seconds
#         self.timer = self.create_timer(time_period, self.on_timer)
#
#     def on_timer(self):
#         from_frame_rel = self.target_frame
#         to_frame_rel = 'fk_link_mast'
#
#         try:
#             now = Time()
#             trans = self.tf_buffer.lookup_transform(
#                 to_frame_rel,
#                 from_frame_rel,
#                 now)
#         except TransformException as ex:
#             self.get_logger().info(
#                 f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
#             return
#
#         self.get_logger().info(
#                         f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is: {trans}')
#

def main():
    # rclpy.init(args=None)

    node = MotionNode()
    node.main()

    command = JointCommand()
    rclpy.spin_once(command)
    command.execute_testcase()
    rclpy.spin(command)

    # notes: we can now use command.GetCurrentPos(self, joint_name) to get current position/orientation of any joint
    # (ex. for joint_wrist_yaw, it returns an angle btwn 0 to 3.14)
    # we can use this info to define a PoseStamped and publish it (see file PosePublisher which publishes something evert )

    # however we need to figure out how to convert btwn this position/orientation of the joint (single number) + frame id (which frame) to 
    # a Cartesian Pose position/orientation (3d angle and 4d quaternion) and vice versa using forward + inverse kinematics


    node.destroy_node()
    command.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
