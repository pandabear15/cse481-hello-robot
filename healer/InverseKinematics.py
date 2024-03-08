#!/usr/bin/env python3

import ikpy.urdf.utils
import pathlib
import stretch_body.hello_utils as hu
from IPython import display
import ipywidgets as widgets

import numpy as np
import urdfpy
import numpy as np

import ikpy.chain

class InverseKinematics():
    def __init__(self, read_write_api, base):
        self.read_write_api = read_write_api
        self.base = base
        
        # setup inverse kinematics. start by getting tree of all links
        urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        modified_urdf = urdfpy.URDF.load(urdf_path)
        #tree = ikpy.urdf.utils.get_urdf_tree(urdf_path, "base_link")[0]
        # #display.display_svg(tree)
        # original_urdf = urdfpy.URDF.load(urdf_path)
        # print(f"name: {original_urdf.name}")
        # print(f"num links: {len(original_urdf.links)}")
        # print(f"num joints: {len(original_urdf.joints)}")
        
        # reduce tree of links to a chain of links
        names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 
                                    'link_gripper_finger_left', 'link_gripper_fingertip_left', 
                                    'link_gripper_finger_right', 'link_gripper_fingertip_right', 
                                    'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 
                                    'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 
                                    'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 
                                    'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 
                                    'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 
                                    'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 
                                    'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 
                                    'laser', 'respeaker_base', 
                                    'imu_mobile_base']
        
        names_of_joints_to_remove2= ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_gripper_finger_left', 
                                     'joint_gripper_fingertip_left', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 
                                     'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 
                                     'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 
                                     'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 
                                     'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 
                                     'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 
                                     'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 
                                     'camera_gyro_joint', 'camera_gyro_optical_joint', 'joint_laser', 'joint_respeaker',
                                     'joint_base_imu']
        
        links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
        for lr in links_to_remove:
            modified_urdf._links.remove(lr)
        
        joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove2]
        for jr in joints_to_remove:
            modified_urdf._joints.remove(jr)
            
        # print(f"name: {modified_urdf.name}")
        # print(f"num links: {len(modified_urdf.links)}")
        # print(f"num joints: {len(modified_urdf.joints)}")
        
        # joint_base_translation = urdfpy.Joint(name='joint_base_translation',
        #                               parent='base_link',
        #                               child='link_base_translation',
        #                               joint_type='prismatic',
        #                               axis=np.array([1.0, 0.0, 0.0]),
        #                               origin=np.eye(4, dtype=np.float64),
        #                               limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.0, upper=1.0))
        
        # modified_urdf._joints.append(joint_base_translation)
        # link_base_translation = urdfpy.Link(name='link_base_translation',
        #                                     inertial=None,
        #                                     visuals=None,        
        #                                     collisions=None)
        # modified_urdf._links.append(link_base_translation)

        # # amend the chain
        # for j in modified_urdf._joints:
        #     if j.name == 'joint_mast':
        #         j.parent = 'link_base_translation'
        # print(f"name: {modified_urdf.name}")
        # print(f"num links: {len(modified_urdf.links)}")
        # print(f"num joints: {len(modified_urdf.joints)}")

        # save chain
        iktuturdf_path = "./stretch.urdf"
        modified_urdf.save(iktuturdf_path)
        tree = ikpy.urdf.utils.get_urdf_tree(iktuturdf_path, "base_link")[0]
        display.display_png(tree)
        
        chain = ikpy.chain.Chain.from_urdf_file(iktuturdf_path)
        self.chain = chain
        print(chain.links)
    
    def execute(self, target_point):
        self.tool = 'tool_stretch_dex_wrist'
        
        # dont execute any movement if joint_state data is still unavailable 
        # otherwise reading joint data using the read_write_api might throw errors
        joint_state = self.read_write_api.read()
        if joint_state == None:
            return
        
        q_init = self.get_current_configuration()
        print (self.chain)
        print(q_init)
        print(target_point)
        q_soln = self.chain.inverse_kinematics(target_point, initial_position=q_init)
        self.move_to_configuration(q_soln)
        
    def get_current_configuration(self):
        chain = self.chain
        tool = self.tool
        
        def bound_range(name, value):
            names = [l.name for l in chain.links]
            index = names.index(name)
            bounds = chain.links[index].bounds
            return min(max(value, bounds[0]), bounds[1])

        # do reads using joint api
        read_write_api = self.read_write_api
       
        if tool == 'tool_stretch_gripper':
            return
        
        elif tool == 'tool_stretch_dex_wrist':
            q_base = 0.0
            
            print(read_write_api.readCurrentPos('joint_lift'))
            print(read_write_api.readCurrentPos('joint_arm_l0'))
            print(read_write_api.readCurrentPos('joint_wrist_yaw'))
            print(read_write_api.readCurrentPos('joint_wrist_pitch'))
            print(read_write_api.readCurrentPos('joint_wrist_roll'))
            
            q_lift = bound_range('joint_lift', read_write_api.readCurrentPos('joint_lift'))
            q_arml = bound_range('joint_arm_l0', read_write_api.readCurrentPos('joint_arm_l0') / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', read_write_api.readCurrentPos('joint_wrist_yaw'))
            q_pitch = bound_range('joint_wrist_pitch', read_write_api.readCurrentPos('joint_wrist_pitch'))
            q_roll = bound_range('joint_wrist_roll', read_write_api.readCurrentPos('joint_wrist_roll'))
            
            print ([0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0])
            return [0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

    def move_to_configuration(self, q):
        tool = self.tool
        
        if tool == 'tool_stretch_gripper':
            return
            
        elif tool == 'tool_stretch_dex_wrist':
            use_init_pos = True
            joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
            seconds = [0.0, 0.02] 
            
            print(q)
            # q_base = q[1]
            # q_lift = q[3]
            # q_arm = q[5] + q[6] + q[7] + q[8]
            # q_yaw = q[9]
            # q_pitch = q[11]
            # q_roll = q[12]
            
            q_lift = q[2]
            q_arm =  q[4] + q[5] + q[6] + q[7]
            q_yaw = q[8]
            q_pitch = q[10]
            q_roll = q[11]
            
            # self.base.translate_by(q_base)
            # self.base.push_command()
            
            # base_end_pos = self.read_write_api.readCurrentPos('joint_mast') + q_base
            positions = [[q_lift, q_arm, q_yaw, q_pitch, q_roll]]
            
            self.read_write_api.write(joint_names, use_init_pos, seconds, positions)
