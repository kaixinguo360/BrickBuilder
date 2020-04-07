# -*- coding: utf-8 -*-

###########################################################################
## Python code generated with wxFormBuilder (version Feb 16 2016)
## http://www.wxformbuilder.org/
##
## PLEASE DO "NOT" EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc


###########################################################################
## Class MyFrame
###########################################################################

class MyFrame(wx.Frame):

    def __init__(self, parent):
        wx.Frame.__init__(self, parent, id=wx.ID_ANY, title=wx.EmptyString, pos=wx.DefaultPosition,
                          size=wx.Size(810, 636), style=wx.DEFAULT_FRAME_STYLE | wx.TAB_TRAVERSAL)

        self.SetSizeHintsSz(wx.DefaultSize, wx.DefaultSize)

        mainSizer = wx.GridBagSizer(0, 0)
        mainSizer.SetFlexibleDirection(wx.BOTH)
        mainSizer.SetNonFlexibleGrowMode(wx.FLEX_GROWMODE_SPECIFIED)

        self.draw_panel = wx.Panel(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL)
        self.draw_panel.SetMinSize(wx.Size(600, 500))

        mainSizer.Add(self.draw_panel, wx.GBPosition(0, 0), wx.GBSpan(1, 1), wx.EXPAND | wx.ALL, 5)

        sideSizer = wx.GridSizer(0, 1, 0, 0)

        sideSizer.SetMinSize(wx.Size(200, 100))
        self.list_panel = wx.Panel(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL)
        listSizer = wx.BoxSizer(wx.VERTICAL)

        listBoxChoices = []
        self.listBox = wx.ListBox(self.list_panel, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, listBoxChoices, 0)
        listSizer.Add(self.listBox, 1, wx.EXPAND | wx.ALL, 5)

        self.list_panel.SetSizer(listSizer)
        self.list_panel.Layout()
        listSizer.Fit(self.list_panel)
        sideSizer.Add(self.list_panel, 1, wx.EXPAND | wx.TOP | wx.RIGHT, 5)

        self.prop_panel = wx.Panel(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL)
        propGroupSizer = wx.StaticBoxSizer(wx.StaticBox(self.prop_panel, wx.ID_ANY, u"属性"), wx.VERTICAL)

        propSizer = wx.FlexGridSizer(0, 2, 0, 0)
        propSizer.SetFlexibleDirection(wx.BOTH)
        propSizer.SetNonFlexibleGrowMode(wx.FLEX_GROWMODE_SPECIFIED)

        self.m_staticText20 = wx.StaticText(propGroupSizer.GetStaticBox(), wx.ID_ANY, u"名称", wx.DefaultPosition,
                                            wx.DefaultSize, 0)
        self.m_staticText20.Wrap(-1)
        propSizer.Add(self.m_staticText20, 0, wx.ALL, 5)

        self.textBox_name = wx.TextCtrl(propGroupSizer.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition,
                                        wx.DefaultSize, wx.TE_PROCESS_ENTER)
        propSizer.Add(self.textBox_name, 0, wx.ALL, 5)

        self.m_staticText14 = wx.StaticText(propGroupSizer.GetStaticBox(), wx.ID_ANY, u"横向位置", wx.DefaultPosition,
                                            wx.DefaultSize, 0)
        self.m_staticText14.Wrap(-1)
        propSizer.Add(self.m_staticText14, 0, wx.ALL, 5)

        self.textBox_X = wx.TextCtrl(propGroupSizer.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition,
                                     wx.DefaultSize, wx.TE_PROCESS_ENTER)
        propSizer.Add(self.textBox_X, 0, wx.ALL, 5)

        self.m_staticText15 = wx.StaticText(propGroupSizer.GetStaticBox(), wx.ID_ANY, u"纵向位置", wx.DefaultPosition,
                                            wx.DefaultSize, 0)
        self.m_staticText15.Wrap(-1)
        propSizer.Add(self.m_staticText15, 0, wx.ALL, 5)

        self.textBox_Y = wx.TextCtrl(propGroupSizer.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition,
                                     wx.DefaultSize, wx.TE_PROCESS_ENTER)
        propSizer.Add(self.textBox_Y, 0, wx.ALL, 5)

        self.m_staticText16 = wx.StaticText(propGroupSizer.GetStaticBox(), wx.ID_ANY, u"旋转角度", wx.DefaultPosition,
                                            wx.DefaultSize, 0)
        self.m_staticText16.Wrap(-1)
        propSizer.Add(self.m_staticText16, 0, wx.ALL, 5)

        self.textBox_R = wx.TextCtrl(propGroupSizer.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition,
                                     wx.DefaultSize, wx.TE_PROCESS_ENTER)
        propSizer.Add(self.textBox_R, 0, wx.ALL, 5)

        propSizer.AddSpacer((0, 0), 1, wx.EXPAND, 5)

        self.button_rotate = wx.Button(propGroupSizer.GetStaticBox(), wx.ID_ANY, u"旋转", wx.DefaultPosition,
                                       wx.DefaultSize, 0)
        propSizer.Add(self.button_rotate, 0, wx.ALL, 5)

        propSizer.AddSpacer((0, 0), 1, wx.EXPAND, 5)

        self.is_aligned = wx.CheckBox(propGroupSizer.GetStaticBox(), wx.ID_ANY, u"对齐到网格", wx.DefaultPosition,
                                      wx.DefaultSize, 0)
        self.is_aligned.SetValue(True)
        propSizer.Add(self.is_aligned, 0, wx.ALL, 5)

        propGroupSizer.Add(propSizer, 1, wx.EXPAND, 5)

        self.prop_panel.SetSizer(propGroupSizer)
        self.prop_panel.Layout()
        propGroupSizer.Fit(self.prop_panel)
        sideSizer.Add(self.prop_panel, 1, wx.EXPAND | wx.RIGHT, 5)

        mainSizer.Add(sideSizer, wx.GBPosition(0, 1), wx.GBSpan(1, 1), wx.EXPAND, 5)

        self.opt_panel = wx.Panel(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL)
        sbSizer4 = wx.StaticBoxSizer(wx.StaticBox(self.opt_panel, wx.ID_ANY, u"操作"), wx.HORIZONTAL)

        sbSizer4.SetMinSize(wx.Size(-1, 80))

        sbSizer4.AddSpacer((0, 0), 1, wx.EXPAND, 5)

        self.button_run = wx.Button(sbSizer4.GetStaticBox(), wx.ID_ANY, u"运行", wx.DefaultPosition, wx.DefaultSize, 0)
        sbSizer4.Add(self.button_run, 0, wx.ALL, 5)

        self.button_add = wx.Button(sbSizer4.GetStaticBox(), wx.ID_ANY, u"添加", wx.DefaultPosition, wx.DefaultSize, 0)
        sbSizer4.Add(self.button_add, 0, wx.ALL, 5)

        self.button_remove = wx.Button(sbSizer4.GetStaticBox(), wx.ID_ANY, u"删除", wx.DefaultPosition, wx.DefaultSize, 0)
        sbSizer4.Add(self.button_remove, 0, wx.ALL, 5)

        self.opt_panel.SetSizer(sbSizer4)
        self.opt_panel.Layout()
        sbSizer4.Fit(self.opt_panel)
        mainSizer.Add(self.opt_panel, wx.GBPosition(1, 0), wx.GBSpan(1, 2), wx.EXPAND | wx.RIGHT | wx.LEFT, 5)

        self.SetSizer(mainSizer)
        self.Layout()
        self.status_bar = self.CreateStatusBar(1, wx.ST_SIZEGRIP, wx.ID_ANY)

        self.Centre(wx.BOTH)

    def __del__(self):
        pass


