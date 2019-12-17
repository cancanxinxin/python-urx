# robot control
# coding=utf-8 
'''
include 12 changes of the pose array 
By now, the following functions are confirmed to be OK 18.12.29
'''
'''
UR5 robot's joints values are shown as follow:
from down to up
base_joint:joint1
shoulder_joint:joint2
elbow_joint:joint3
wrist_1_joint:joint4
wrist_2_joint:joint5
wrist_3_joint:joint6

1 rad = (180/pi) degree
[joint1,joint2,joint3,joint4,joint5,joint6]
degree = [-100.76,-154,103.96,-35,90,8.39]
joints =  [-1.7585955482160456, -2.7457519858355792, 1.9045014922636971, -0.6447718290487824, 1.5750147740326217, 0.1464862489863563]


'''
import numpy as np
from matplotlib import pylab as plt
import argparse
import glob
from math import pi
import urx
import logging
import time
import sys
import math3d as m3d  

delta_joint_value = pi/180

def print_robot_joints():
    # print robot joint value
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joints = rob.getj()
    # print("robot current tcp is at: ", pose)    #   in the world coordinate position and rotation    
    print("joints = ", joints)
    rob.close()

# rob.movej(initj, acc=0.8, vel=0.2)
# move the robot in the joint value
def robot_move2joints():
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    # joints = rob.getj()
    # print("joints = ", joints)
    targetj = [-1.7585955482160456, -2.688812721821277, 1.813553548211062, -0.6114363053187155, 1.5750147740326217, 0.1464862489863563]
    rob.movej(targetj, acc=0.8, vel=0.2)
    rob.close()


def print_robot_tcp():
    # ptint robot current tcp
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    pose = rob.getl()
    # print("robot current tcp is at: ", pose)    #   in the world coordinate position and rotation    
    print("pose = ", pose)
    rob.close() 

def print_delta_robot_tcp():
    # print delta_robot_tcp related to dui1 position
    # ptint robot current tcp
    pose_dui1 =  [-0.015379972627099047, 0.16278659829816503, 0.35778023133243914, 0.7293855033267473, 1.7571449945285949, 1.7263708455148004]
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    pose = rob.getl()
    # print("robot current tcp is at: ", pose)    #   in the world coordinate position and rotation    
    print("pose = ", pose)
    delta_robot_tcp = np.array(pose) - np.array(pose_dui1)
    print("delta_robot_tcp = ", delta_robot_tcp)
    return(delta_robot_tcp)

def test_delta_robot_tcp(delta_robot_tcp):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    pose = np.array(pose) + np.array(delta_robot_tcp)
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close() 

# get_Transformation() is OK to be used
def get_Transformation():
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    j = rob.getj()
    print("Initial joint configuration is ", j) # initial six joint states    
    t = rob.get_pose()
    print("Transformation from base to tcp is: ", t)
    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation
    rob.close()

#  it's good enough to use
'''
changes of pose[0,1,2,4,5,6]
robot_move0_left()
robot_move0_right()
robot_move1_forward()
robot_move1_back()
robot_move2_up()
robot_move2_down()
robot_move3_roll_add()
robot_move3_roll_sub()
robot_move4_pitch_add()
robot_move4_pitch_sub()
robot_move5_yaw_add()
robot_move5_yaw_sub()

changes of pose[3,4,5] to be completed
roll+
roll-
pitch+
pitch-
yaw+
yaw-
'''

# pose[0]
def robot_move0_left():
    # to move left
    print("moving left")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction down for 0.02
    # print("moving in x")
    pose[0] -= 0.005
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move0_left_less():
    # to move left
    print("moving left")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction down for 0.02
    # print("moving in x")
    pose[0] -= 0.003
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move0_left_N(N):
    # to move left
    print("moving left")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction down for 0.02
    # print("moving in x")
    pose[0] -= 0.005*N
    rob.movel(pose, acc=0.5, vel=0.1)
    rob.close()

def robot_move0_right():
    # to move right
    print("moving right")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction  for 0.02
    # print("moving in x")
    pose[0] += 0.005
    rob.movel(pose, acc=0.1, vel=0.02)
    rob.close()

def robot_move0_right_N(N):
    # to move right
    print("moving right")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    pose[0] += 0.005*N
    rob.movel(pose, acc=0.1, vel=0.02)
    rob.close()

def robot_move0_right_N1(N):
    # to move right
    print("moving right")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction  for 0.02
    # print("moving in x")
    pose[0] += 0.005*N
    rob.movel(pose, acc=0.25, vel=0.05)
    rob.close()

# pose[1]
def robot_move1_forward():
    # to move forward
    print("moving forward")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in y direction  for 0.02
    # print("moving in y")
    pose[1] += 0.01
    rob.movel(pose, acc=0.1, vel=0.02)
    rob.close()

def robot_move1_forward_N(N):
    # to move forward
    print("moving forward")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in y direction  for 0.02
    # print("moving in y")
    pose[1] += 0.005*N
    rob.movel(pose, acc=0.5, vel=0.1)
    rob.close()

def robot_move1_back():
    # to move back
    print("moving back")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction  for 0.02
    # print("moving in x")
    pose[1] -= 0.01
    rob.movel(pose, acc=0.1, vel=0.02)
    rob.close()

def robot_move1_back_N(N):
    # to move back
    print("moving back")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in x direction  for 0.02
    # print("moving in x")
    pose[1] -= 0.005*N
    rob.movel(pose, acc=0.5, vel=0.1)
    rob.close()

# pose[2]
def robot_move2_up():
    # to move up
    print("moving up")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction up for 0.02
    # print("moving in z")
    pose[2] += 0.005
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move2_up_N(N):
    # to move up
    print("moving up")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction up for 0.02
    # print("moving in z")
    pose[2] += 0.005*N
    rob.movel(pose, acc=0.5, vel=0.1)
    rob.close()

def robot_move2_up_less():
    # to move up
    print("moving up")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction up for 0.02
    # print("moving in z")
    pose[2] += 0.003
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()


def robot_move2_down():
    # to move down
    print("moving down")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[2] -= 0.005
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move2_down_N(N):
    # to move down
    print("moving down")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[2] -= 0.005*N
    rob.movel(pose, acc=0.5, vel=0.1)
    rob.close()

def robot_move3_roll_add():
    # to roll add
    print("rolling add")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[3] += pi/90
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move3_roll_sub():
    # to roll add
    print("rolling sub")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[3] -= pi/90
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move4_pitch_add():
    # to roll add
    print("pitching add")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[4] += pi/90
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move4_pitch_sub():
    # to pitch add
    print("pitching sub")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[4] -= pi/90
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move5_yaw_add():
     # to yaw add
    print("yawing sub")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[5] += pi/90
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

def robot_move5_yaw_sub():
    # to yaw add
    print("yawing sub")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in z direction down for 0.02
    # print("moving in z")
    pose[5] -= pi/90
    rob.movel(pose, acc=0.1, vel=0.01)
    rob.close()

# move the robot to dui1
def move_to_dui1():
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    v = 0.02
    a = 0.1      
    #dui1 pose start
    pose = [-0.015371047712964686, 0.16291898650384315, 0.3583823679602556, 0.729218924720313, 1.756045637777791, 1.72787322187088]
    rob.movel(pose, acc=a, vel=v)
    rob.close()

'''
至此，完成机械臂可能的pose的6个数值的加减操作，12种运动

'''

def robot_move1_forward_less():
    # to move forward
    print("moving forward")
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    pose = rob.getl()
    print("robot tcp is at: ", pose)    #   in the world coordinate position and rotation

    # move in y direction  for 0.02
    # print("moving in y")
    pose[1] += 0.005
    rob.movel(pose, acc=0.1, vel=0.02)
    rob.close()

def robot_move_joint1_add(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint1 add 180/pi")
    joint_pose[0] += delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint1_sub(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint1 sub 180/pi")
    joint_pose[0] -= delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint2_add(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint2 add 180/pi")
    joint_pose[1] += delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint2_sub(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint2 sub 180/pi")
    joint_pose[1] -= delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint3_add(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint3 add 180/pi")
    joint_pose[2] += delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint3_sub(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint3 sub 180/pi")
    joint_pose[2] -= delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint4_add(N):    #wrist 1
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint4 add 180/pi")
    joint_pose[3] += delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint4_sub(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint4 sub 180/pi")
    joint_pose[3] -= delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint5_add(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint5 add 180/pi")
    joint_pose[4] += delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint5_sub(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint5 sub 180/pi")
    joint_pose[4] -= delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint6_add(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint6 add 180/pi")
    joint_pose[5] += delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

def robot_move_joint6_sub(N):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    joint_pose = rob.getj()
    # print("joints = ", joints)
    print("joint6 sub 180/pi")
    joint_pose[5] -= delta_joint_value*N
    rob.movej(joint_pose, acc=0.8, vel=0.2)
    rob.close()

# print(delta_joint_value)
# pose_tie_back_20 =  [-0.1491649063910727, -0.04348820779175313, 0.4261605769153223, -0.01718198766360668, -2.325400869114534, -2.0537595422226045]
# move the robot to open1 back 20 times
def move_to_tie_back40():
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    v = 0.02
    a = 0.1      
    #dui1 pose start
    # pose = [-0.1491649063910727, -0.04348820779175313, 0.4261605769153223, -0.01718198766360668, -2.325400869114534, -2.0537595422226045]
    pose = [-0.14939671177213512, -0.14343155569388938, 0.4255669998998176, -0.01813156630004468, -2.3270854953626006, -2.0522019537574065]
    rob.movel(pose, acc=a, vel=v)
    rob.close() 
    
def move_to_pose_target(pose_target):
    rob = urx.Robot("192.168.80.2")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    v = 0.02
    a = 0.1      
    rob.movel(pose_target, acc=a, vel=v)
    rob.close() 
# demo start
# demo_start1 = [-0.1497819800013426, -0.13449279034770753, 0.42645890036500606, -0.015308508906232242, -2.323463252654852, -2.0578295936562165]
