#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('/home/shlok/RRT/build-rrtplugin/librrtplugin.so')
try:
    env=Environment()
    env.Load('include/scenes/myscene.env.xml')
    RRTPlugin = RaveCreateModule(env,'RRTPlugin')
    print RRTPlugin.SendCommand('help')
finally:
    RaveDestroy()
