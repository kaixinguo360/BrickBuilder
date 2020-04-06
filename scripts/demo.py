#!/usr/bin/python2
# coding=utf-8
import rospy
from arm import Arm
from builder import Builder
from scene import Scene
from utils import *

init('moveit_demo')

scene = Scene('world')
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

arm = Arm('panda_arm', 'world')
arm.cartesian = True
arm.gripper_open_posture = 0.09
arm.gripper_close_posture = 0.05

builder = Builder(arm, scene)
builder.base_offset = build_frame((0, 0.5, 0.0))
builder.hand_offset = build_frame((0, 0, -0.18), (0, 0, pi/4))
builder.load_from_path(builder.get_path() + '/config/demo.json')
builder.show_target()
rospy.sleep(5)
builder.prepare_scene()
builder.build()

rospy.spin()

