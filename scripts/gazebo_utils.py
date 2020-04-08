#!/usr/bin/python2
# coding=utf-8
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel
from rospy import ServiceProxy

from utils import frame_to_pose


class GazeboUtils:

    spawn_model = None  # type: ServiceProxy
    delete_model = None  # type: ServiceProxy

    def __init__(self):
        rospy.wait_for_service('gazebo/spawn_urdf_model')
        rospy.wait_for_service('gazebo/delete_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        pass

    def spawn_bricks(self, name, size, transform):
        rospy.loginfo('SPAWN BRICK [' + name + ']')

        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml = create_brick(name, size)
        req.initial_pose = frame_to_pose(transform)

        # should this be unique so ros_control can use each individually?
        req.robot_namespace = "/bricks"

        self.spawn_model(req)

    def delete_bricks(self, name):
        rospy.loginfo('DELETE BRICK [' + name + ']')
        self.delete_model(name)


def create_brick(name, size):
    visual = (
        '<visual>'
        '  <origin xyz="0 0 0" rpy="0 0 0"/>'
        '  <geometry>'
        '    <box size="{} {} {}"/>'
        '  </geometry>'
        '  <material name="yellow">'
        '    <color rgba="1 1 0 1"/>'
        '  </material>'
        '</visual>'
    ).format(size[0], size[1], size[2])

    collision = (
        '<collision>'
        '  <geometry>'
        '    <box size="{} {} {}"/>'
        '  </geometry>'
        '</collision>'
    ).format(size[0], size[1], size[2])

    mass = 0.01
    radius = (size[0] + size[1]) / 4
    length = size[2]
    inertial = (
        '<inertial>'
        '  <origin xyz="0 0 0" rpy="0 0 0"/>'
        '  <mass value="{}" />'
        '  <inertia ixx="{}" ixy="0.0" ixz="0.0"'
        '           iyy="{}" iyz="0.0"'
        '           izz="{}" />'
        '</inertial>'
    ).format(
        mass,
        0.0833333 * mass * (3 * radius * radius + length * length),
        0.0833333 * mass * (3 * radius * radius + length * length),
        0.5 * mass * radius * radius,
    )

    link = '<link name="{}">{}{}{}</link>'.format(name, visual, collision, inertial)

    gazebo = (
        '<gazebo reference="{}">'
        '  <material>Gazebo/Yellow</material>'
        '  <mu1>1.0</mu1>'
        '  <mu2>1.0</mu2>'
        '  <minDepth>0.01</minDepth>'
        '  <maxVel>0</maxVel>'
        '</gazebo>'
    ).format(name)

    model = (
        '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '{}'
        '{}'
        '</robot>'
    ).format(name, link, gazebo)

    return model
