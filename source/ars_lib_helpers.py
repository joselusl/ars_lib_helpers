#!/usr/bin/env python

import numpy as np
from numpy import *

import os



# ROS

import rospy

import tf_conversions as tf





class Quaternion:

  @staticmethod
  def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
      return v
    return v / norm

  @staticmethod
  def zerosQuat():
    return Quaternion.normalize(np.array([1.0, 0.0, 0.0, 0.0], dtype=float))

  @staticmethod
  def zerosQuatSimp():
    return Quaternion.normalize(np.array([1.0, 0.0], dtype=float))

  @staticmethod
  def setQuatSimp(v):
    return Quaternion.normalize(v)


  @staticmethod
  def getSimplifiedQuatRobotAtti(robot_atti_quat):

    robot_atti_quat_tf = np.roll(robot_atti_quat, -1)
    robot_atti_ang = tf.transformations.euler_from_quaternion(robot_atti_quat_tf, axes='sxyz')
    robot_atti_ang_yaw = robot_atti_ang[2]

    robot_atti_hor_quat_tf = tf.transformations.quaternion_from_euler(0, 0, robot_atti_ang_yaw, axes='sxyz')
    robot_atti_hor_quat = np.roll(robot_atti_hor_quat_tf, 1)
    robot_atti_quat_simp = Quaternion.zerosQuatSimp()
    robot_atti_quat_simp[0] = robot_atti_hor_quat[0]
    robot_atti_quat_simp[1] = robot_atti_hor_quat[3]
    robot_atti_quat_simp = Quaternion.normalize(robot_atti_quat_simp)

    return robot_atti_quat_simp




class Pose:

  position = np.zeros((3,), dtype=float)

  attitude_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

  def __init__(self):

    self.position = np.zeros((3,), dtype=float)

    self.attitude_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    return



class PoseSimp:

  parent_frame = ''
  child_frame = ''

  position = np.zeros((3,), dtype=float)

  attitude_quat_simp = np.array([1.0, 0.0], dtype=float)

  def __init__(self):

    parent_frame = ''
    child_frame = ''

    self.position = np.zeros((3,), dtype=float)

    self.attitude_quat_simp = np.array([1.0, 0.0], dtype=float)

    return



class PoseAlgebra:

  @staticmethod
  def computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2):

    error_quat_simp = Quaternion.zerosQuatSimp()
    error_quat_simp[0] = atti_quat_simp_1[0]*atti_quat_simp_2[0]+atti_quat_simp_1[1]*atti_quat_simp_2[1]
    error_quat_simp[1] = atti_quat_simp_1[1]*atti_quat_simp_2[0]-atti_quat_simp_1[0]*atti_quat_simp_2[1]
    if(error_quat_simp[0] < 0):
      error_quat_simp = -1 * error_quat_simp

    return error_quat_simp


  @staticmethod
  def computeScalarDiffFromDiffQuatSimp(delta_atti_quat_simp):

    if(delta_atti_quat_simp[0] < 0):
      delta_atti_quat_simp = -1 * delta_atti_quat_simp

    error_att = 2.0 * delta_atti_quat_simp[1].item()

    return error_att



  @staticmethod
  def computePoseSimpDifference(posi_1, atti_quat_simp_1, posi_2, atti_quat_simp_2):

    # Position
    delta_posi = posi_1 - posi_2

    # Attitude
    delta_atti_quat_simp = PoseAlgebra.computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2)


    # End
    return delta_posi, delta_atti_quat_simp




class Conversions:

  @staticmethod
  def convertVelLinFromRobotToWorld(robot_velo_lin_robot, robot_atti_quat_in, flag_quat_simp=True):

    robot_atti_quat = np.zeros((4,), dtype=float)

    if(flag_quat_simp):
      robot_atti_quat[0] = robot_atti_quat_in[0]
      robot_atti_quat[3] = robot_atti_quat_in[1]
    else:
      robot_atti_quat = robot_atti_quat_in
    robot_atti_quat = Quaternion.normalize(robot_atti_quat)

    robot_atti_quat_tf = np.roll(robot_atti_quat, -1)
    robot_atti_ang = tf.transformations.euler_from_quaternion(robot_atti_quat_tf, axes='sxyz')
    robot_atti_ang_yaw = robot_atti_ang[2]

    robot_velo_lin_world = np.zeros((3,), dtype=float)

    robot_velo_lin_world[0] = math.cos(robot_atti_ang_yaw)*robot_velo_lin_robot[0]-math.sin(robot_atti_ang_yaw)*robot_velo_lin_robot[1]
    robot_velo_lin_world[1] = math.sin(robot_atti_ang_yaw)*robot_velo_lin_robot[0]+math.cos(robot_atti_ang_yaw)*robot_velo_lin_robot[1]
    robot_velo_lin_world[2] = robot_velo_lin_robot[2]

    return robot_velo_lin_world

  @staticmethod
  def convertVelAngFromRobotToWorld(robot_velo_ang_robot, robot_atti_quat_in, flag_quat_simp=True):

    return robot_velo_ang_robot




  @staticmethod
  def convertVelLinFromWorldToRobot(robot_velo_lin_world, robot_atti_quat_in, flag_quat_simp=True):

    robot_atti_quat = np.zeros((4,), dtype=float)

    if(flag_quat_simp):
      robot_atti_quat[0] = robot_atti_quat_in[0]
      robot_atti_quat[3] = robot_atti_quat_in[1]
    else:
      robot_atti_quat = robot_atti_quat_in
    robot_atti_quat = Quaternion.normalize(robot_atti_quat)

    robot_atti_quat_tf = np.roll(robot_atti_quat, -1)
    robot_atti_ang = tf.transformations.euler_from_quaternion(robot_atti_quat_tf, axes='sxyz')
    robot_atti_ang_yaw = robot_atti_ang[2]

    robot_velo_lin_robot = np.zeros((3,), dtype=float)

    robot_velo_lin_robot[0] = math.cos(robot_atti_ang_yaw)*robot_velo_lin_world[0]+math.sin(robot_atti_ang_yaw)*robot_velo_lin_world[1]
    robot_velo_lin_robot[1] = -math.sin(robot_atti_ang_yaw)*robot_velo_lin_world[0]+math.cos(robot_atti_ang_yaw)*robot_velo_lin_world[1]
    robot_velo_lin_robot[2] = robot_velo_lin_world[2]

    return robot_velo_lin_robot

  @staticmethod
  def convertVelAngFromWorldToRobot(robot_velo_ang_world, robot_atti_quat_in, flag_quat_simp=True):

    return robot_velo_ang_world




def isPointInCircle(point, circle_center, circle_radius):

  circle_impl_equ = (point[0]-circle_center[0])**2 + (point[1]-circle_center[1])**2 - circle_radius**2

  if(circle_impl_equ < 1.0):
    return True
  else:
    return False



def distanceSegmentCircle(point_segment_1, point_segment_2, circle_center, circle_radius):

  # TODO

  return 0.0