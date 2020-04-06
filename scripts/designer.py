import wx
from wx.lib.floatcanvas.FloatCanvas import FloatCanvas

from MyFrame import MyFrame

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

        self.canvas = FloatCanvas(
            self.draw_panel, size=(600, 500),
            ProjectionFun=None,
            BackgroundColor="White"
        )

        self.canvas.Bind(wx.EVT_LEFT_DOWN,self.on_mouse)
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
        new_brick = Brick(name)
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


# def showMessage(self, event):
# self.m_textCtrl1.Clear()
# self.m_textCtrl1.SetValue('hello world')

if __name__ == '__main__':
    app = wx.App()

    main_win = MainWin(None)
    main_win.Show()

    app.MainLoop()
