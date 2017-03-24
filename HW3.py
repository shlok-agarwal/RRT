#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('/home/shlok/RRT/build-rrtplugin/librrtplugin.so')
    ### END INITIALIZING YOUR PLUGIN ###


    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    startconfig = [-0.15,0.075,-1.008,-0.11,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)
    ### these two function are part of my code
    draw=[]
    def splitPath(path):
        path = path.split(',')
        return path

    def drawRawPath(path, color):
        sizeRawPath=int(path[0])
        k=2
        for i in range(sizeRawPath):
            config=[]
            config.append(float(path[k]))
            config.append(float(path[k+1]))
            config.append(float(path[k+2]))
            config.append(float(path[k+3]))
            config.append(float(path[k+4]))
            config.append(float(path[k+5]))
            config.append(float(path[k+6]))
            k=k+7
            robot.SetActiveDOFValues(config)
            draw.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.01,colors=color,drawstyle=1))

    def drawSmoothPath(path, color):
        sizeRawPath=int(path[0])
        sizeSmoothPath=int(path[1])
        k=7*sizeRawPath+2
        for i in range(sizeSmoothPath):
            config=[]
            config.append(float(path[k]))
            config.append(float(path[k+1]))
            config.append(float(path[k+2]))
            config.append(float(path[k+3]))
            config.append(float(path[k+4]))
            config.append(float(path[k+5]))
            config.append(float(path[k+6]))
            k=k+7
            robot.SetActiveDOFValues(config)
            draw.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.01,colors=color,drawstyle=1))

    with env:
        goalconfig = [0.449,-0.201,-0.151,-0.11,0,-0.11,0]

        ### YOUR CODE HERE ###
        RRTPlugin = RaveCreateModule(env,'RRTPlugin')
        RRTPlugin.SendCommand('Test')
        #RRTPlugin.SendCommand('SetConfigDimension'+' '+str(len(goalconfig)))
        #RRTPlugin.SendCommand('SetStartConfig'+' '+str(startconfig).translate(None, "[],"))
        #RRTPlugin.SendCommand('SetGoalConfig'+' '+str(goalconfig).translate(None, "[],"))
        path=RRTPlugin.SendCommand('StartRRT'+' '+str(startconfig).translate(None, "[],")+' '+str(goalconfig).translate(None, "[],"))
        drawRawPath(splitPath(path), [1, 0, 0])
        drawSmoothPath(splitPath(path),[0, 0, 1])
        
        #print str([int(robot.GetJointFromDOFIndex(x).IsCircular(0)) for x in robot.GetActiveDOFIndices()]).translate(None, "[],")
        #print robot.GetActiveDOFIndices()
        #print str(startconfig).translate(None, "[],")+' '+str(goalconfig).translate(None, "[],")




        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")
