#!/usr/bin/python2
# coding=utf-8
import json
import rospkg

from scene import Scene
from arm import Arm
from builder import Builder
from utils import *

init('moveit_demo')

# scene #
scene = Scene('world')
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

# arm #
arm = Arm(
    'panda_arm',
    'world',
    build_frame((0, 0, -0.12), (0, 0, pi/4)),
    0.05,
    0.02
)
arm.cartesian = True

# builder #
margin = 0.001
builder = Builder(
    arm, scene,
    (0.2 - margin, 0.05 - margin, 0.1 - margin)
)

rp = rospkg.RosPack()
config_file = open(rp.get_path('brick_builder') + '/config/demo.json', 'r')
builder.set_targets(json.load(config_file))

builder.show_target()
rospy.sleep(2)
builder.show_source()
builder.build()

rospy.spin()

