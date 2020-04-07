#!/usr/bin/python2
# coding=utf-8

from PyKDL import Frame

from arm import Arm
from scene import Scene
from gazebo_utils import GazeboUtils
from utils import build_frame, frame, pi


class Builder:

    # instance #
    arm = None  # type: Arm
    scene = None  # type: Scene
    gazebo_utils = None  # type: GazeboUtils

    # config #
    base_offset = build_frame((0.1, 0.6, 0))  # type: Frame
    brick_size = None  # type: tuple

    # bricks #
    targets = None  # type: list
    sources = []

    def __init__(self, arm, scene, brick_size, gazebo_utils=None):
        self.arm = arm
        self.scene = scene
        self.brick_size = brick_size
        self.gazebo_utils = gazebo_utils

    # ---- public ---- #

    def set_targets(self, targets):
        self.targets = targets
        self.targets.sort(key=lambda t: t.p.z() * 10000 + -t.p.y() * 100 + t.p.x())

    def show_source(self):
        self.reset_scene()
        for y in range(1, -10, -1):
            for x in range(-1, 2, 2):
                if len(self.sources) >= len(self.targets):
                    return
                self.add_brick(build_frame((0.37 * x, y * (self.brick_size[1] * 2 + 0.01), self.brick_size[2] / 2), (pi, 0, 0)))
            for x in range(-1, 2, 2):
                if len(self.sources) >= len(self.targets):
                    return
                self.add_brick(build_frame((0.5 * x, (y - 0.5) * (self.brick_size[1] * 2 + 0.01), self.brick_size[2] / 2), (pi, 0, 0)))

    def show_target(self):
        self.reset_scene()
        for transform in self.targets:
            self.add_brick(self.base_offset * transform)

    def build(self):
        for index in range(0, len(self.targets), 1):
            source = self.sources[index]
            target = self.base_offset * self.targets[index]
            self.from_point_to_point(source, target, index)

    # ---- private ---- #

    def reset_scene(self):
        while len(self.sources) > 0:
            self.sources.pop()
            name = 'brick' + str(len(self.sources))

            self.scene.remove_object(name)
            if self.gazebo_utils is not None:
                self.gazebo_utils.delete_bricks(name)

    def add_brick(self, transform):
        name = 'brick' + str(len(self.sources))
        self.scene.add_box(
            name,
            self.brick_size,
            transform,
            (1, 1, 0, 1)
        )
        if self.gazebo_utils is not None:
            self.gazebo_utils.spawn_bricks(
                name,
                self.brick_size,
                transform
            )
        self.sources.append(transform)

    def from_point_to_point(self, frame1, frame2, index):

        allowed_touch_objects = []
        for i in range(0, index, 1):
            allowed_touch_objects.append('brick' + str(i))

        print allowed_touch_objects

        self.arm.pick(
            'brick' + str(index),
            frame1,
            self.arm.GripperTranslation((0, 0, -1), 0.1, 0.05),
            self.arm.GripperTranslation((0, 0, 1), 0.1, 0.05),
            allowed_touch_objects
        )

        self.arm.place(
            'brick' + str(index),
            frame2,
            self.arm.GripperTranslation((0, 0, -1), 0.1, 0.05),
            self.arm.GripperTranslation((0, -1, 0), 0.1, 0.05),
            allowed_touch_objects
        )
