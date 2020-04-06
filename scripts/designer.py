import wx
from wx.lib.floatcanvas.FloatCanvas import FloatCanvas, Circle, Rectangle, Line, Polygon, Point

import numpy as np
from PyKDL import Frame

from MyFrame import MyFrame
from utils import build_frame


class Brick():
    name = None
    x = None
    y = None
    r = None

    def __init__(self, name, x=0.0, y=0.0, r=0.0):
        self.name = name
        self.x = x
        self.y = y
        self.r = r


class MainWin(MyFrame):
    cur_index = -1
    bricks = []
    bricks_count = 0

    def __init__(self, parent):
        super(MainWin, self).__init__(parent)

        self.Bind(wx.EVT_LISTBOX, self.on_listBox, self.listBox)
        self.Bind(wx.EVT_BUTTON, self.add_brick, self.button_add)
        self.Bind(wx.EVT_BUTTON, self.remove_brick, self.button_remove)
        self.Bind(wx.EVT_BUTTON, self.update_brick, self.button_update)
        self.Bind(wx.EVT_BUTTON, self.run, self.button_run)

        self.canvas = FloatCanvas(
            self.draw_panel, size=(600, 500),
            ProjectionFun=None,
            BackgroundColor="White"
        )

        self.canvas.Bind(wx.EVT_LEFT_DOWN, self.on_mouse)
        self.canvas.Bind(wx.EVT_LEFT_UP, self.on_mouse)

        self.refresh()

    def on_mouse(self, event):
        print 'hello'
        print event.GetEventObject()

    def on_listBox(self, event):
        self.cur_index = event.GetEventObject().GetSelection()
        self.refresh()

    def is_selected(self):
        if 0 <= self.cur_index < len(self.bricks):
            return True
        else:
            self.cur_index = -1
            return False

    # ---- bricks ---- #

    def add_brick(self, event):
        name = 'brick' + str(self.bricks_count)
        new_brick = Brick(name, 0, self.brick_size[1]/2)
        self.bricks.append(new_brick)
        self.cur_index = len(self.bricks) - 1
        self.bricks_count += 1
        self.refresh()

    def remove_brick(self, event):
        if self.is_selected():
            del self.bricks[self.cur_index]
            if self.cur_index >= len(self.bricks):
                self.cur_index = len(self.bricks) - 1
            self.refresh()

    def update_brick(self, event):
        if self.is_selected():
            cur = self.bricks[self.cur_index]

            new_name = self.textBox_name.GetValue()
            if cur.name != new_name:
                for i in range(0, len(self.bricks), 1):
                    if self.bricks[i].name == new_name:
                        return
            cur.name = new_name

            cur.x = float(self.textBox_X.GetValue())
            cur.y = float(self.textBox_Y.GetValue())
            cur.r = float(self.textBox_R.GetValue())
            self.refresh()

    # ---- refresh ---- #

    def refresh(self):
        self.refresh_list()
        self.refresh_prop()
        self.refresh_canvas()

    def refresh_list(self):
        self.listBox.Clear()
        for brick in self.bricks:
            self.listBox.Append(brick.name)
        if self.is_selected():
            self.listBox.Select(self.cur_index)

    def refresh_prop(self):
        if self.is_selected():
            cur = self.bricks[self.cur_index]
            self.textBox_name.SetValue(str(cur.name))
            self.textBox_X.SetValue(str(cur.x))
            self.textBox_Y.SetValue(str(cur.y))
            self.textBox_R.SetValue(str(cur.r))
            # self.prop_panel.Show()
        else:
            self.textBox_name.Clear()
            self.textBox_X.Clear()
            self.textBox_Y.Clear()
            self.textBox_R.Clear()
            # self.prop_panel.Hide()

    # ---- canvas ---- #

    def refresh_canvas(self):
        self.canvas.ClearAll()
        self.draw_lines()

        for brick in self.bricks:
            if not self.is_selected() or brick != self.bricks[self.cur_index]:
                self.canvas.AddObject(self.get_box(brick, "Yellow"))

        if self.is_selected():
            self.canvas.AddObject(self.get_box(self.bricks[self.cur_index], "Yellow", "Orange"))

        self.canvas.Draw()

    def draw_lines(self):
        self.canvas.AddObject(Line([
            self.apply_offset((1, 0)),
            self.apply_offset((-1, 0))
        ], LineWidth=5))
        self.canvas.AddObject(Line([
            self.apply_offset((0, 1)),
            self.apply_offset((0, -1))
        ], LineWidth=2, LineStyle="ShortDash"))

        for x in range(-5, 6, 1):
            self.canvas.AddObject(Line([
                self.apply_offset((x * 0.1, 1)),
                self.apply_offset((x * 0.1, -1))
            ], LineStyle="Dot", LineWidth=0.1))
        for y in range(-5, 10, 1):
            self.canvas.AddObject(Line([
                self.apply_offset((1, y * 0.1)),
                self.apply_offset((-1, y * 0.1))
            ], LineStyle="Dot", LineWidth=0.1))

    scale = 500
    offset = (0, -0.4)
    brick_size = (0.2, 0.05)

    def get_box(self, brick, FillColor, LineColor="Black"):
        brick = ((brick.x + self.offset[0]) * self.scale, (brick.y + self.offset[1]) * self.scale, brick.r)
        brick_size = (
            self.brick_size[0] * self.scale,
            self.brick_size[1] * self.scale
        )
        p1 = to_points(brick, (brick_size[0] / 2, brick_size[1] / 2))
        p2 = to_points(brick, (brick_size[0] / 2, -brick_size[1] / 2))
        p3 = to_points(brick, (-brick_size[0] / 2, -brick_size[1] / 2))
        p4 = to_points(brick, (-brick_size[0] / 2, brick_size[1] / 2))
        return Polygon([p1, p2, p3, p4], LineWidth=2, FillColor=FillColor, LineColor=LineColor)

    def apply_offset(self, point):
        return ((point[0] + self.offset[0]) * self.scale, (point[1] + self.offset[1]) * self.scale)

    # ---- run ---- #

    def run(self):
        pass


def to_points(brick, point):
    a = np.array([
        [point[0]],
        [point[1]
    ]])
    b = np.array([
        [np.cos(brick[2]), -np.sin(brick[2])],
        [np.sin(brick[2]), np.cos(brick[2])]
    ])
    c = np.array([
        [brick[0]],
        [brick[1]]
    ])
    d = b.dot(a) + c
    return (
        d[0][0], d[1][0]
    )


if __name__ == '__main__':
    app = wx.App()

    main_win = MainWin(None)
    main_win.Show()

    app.MainLoop()
