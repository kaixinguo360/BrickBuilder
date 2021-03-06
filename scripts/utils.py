#!/usr/bin/python2
# coding=utf-8
import sys

import moveit_commander
import rospy
from PyKDL import Frame, Vector, Rotation
from geometry_msgs.msg import PoseStamped, Pose

pi = 3.1415926


def init(node_name):
    # 等待MoveIt服务上线
    rospy.wait_for_service('/move_group/load_map')

    # 初始化环境
    moveit_commander.roscpp_initialize(sys.argv)  # 初始化MoveIt
    rospy.init_node(node_name)  # 初始化Ros节点

    rospy.sleep(1)
    rospy.loginfo("Node [" + node_name + "] online.")


def frame_to_pose(frame):
    # type: (Frame) -> Pose
    assert isinstance(frame, Frame)
    pose = Pose()

    assert isinstance(frame.p, Vector)
    vector = frame.p
    pose.position.x = vector.x()
    pose.position.y = vector.y()
    pose.position.z = vector.z()

    assert isinstance(frame.M, Rotation)
    quaternion = frame.M.GetQuaternion()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def pose_to_frame(pose):
    # type: (object) -> Frame
    if isinstance(pose, PoseStamped):
        pose = pose.pose
    assert isinstance(pose, Pose)

    pos = Vector(
        pose.position.x,
        pose.position.y,
        pose.position.z
    )

    rot = Rotation.Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    frame = Frame(rot, pos)

    return frame


def build_frame(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    # type: (tuple, tuple) -> Frame
    pos = Vector(xyz[0], xyz[1], xyz[2])
    rot = Rotation.RPY(rpy[0], rpy[1], rpy[2])
    return Frame(rot, pos)


def frame(config):
    # type: (dict) -> Frame
    xyz = config['xyz'] if config.has_key('xyz') else [0, 0, 0]
    rpy = config['rpy'] if config.has_key('xyz') else [0, 0, 0]
    return build_frame(xyz, rpy)

