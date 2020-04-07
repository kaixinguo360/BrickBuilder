#!/usr/bin/python2
# coding=utf-8
import json
import rospkg
from wx import wx

from scene import Scene
from arm import Arm
from builder import Builder
from designer import Designer
from gazebo_utils import GazeboUtils
from utils import *

init('moveit_demo')
use_gazebo = rospy.get_param("/use_gazebo")

# config #
brick_size = (0.2, 0.05, 0.1)

# scene #
scene = Scene('world')
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

# gazebo #
gazebo = GazeboUtils() if use_gazebo else None

# arm #
arm = Arm(
    'panda_arm',
    'world',
    build_frame((0, 0, -0.12), (0, 0, pi/4)),
    0.03,
    0.025
)
arm.cartesian = True

# builder #
margin = 0.001
builder = Builder(
    arm, scene, gazebo_utils=gazebo,
    brick_size=(brick_size[0] - margin, brick_size[1] - margin, brick_size[2] - margin)
)

# designer #
app = wx.App()
designer = Designer(builder, (brick_size[0], brick_size[1]))
designer.Show()
app.MainLoop()

# rp = rospkg.RosPack()
# config_file = open(rp.get_path('brick_builder') + '/config/demo.json', 'r')
# builder.set_targets(json.load(config_file))
#
# builder.show_target()
# rospy.sleep(2)
# builder.show_source()
# builder.build()

# rospy.spin()

