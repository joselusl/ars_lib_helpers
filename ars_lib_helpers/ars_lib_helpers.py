import numpy as np
from numpy import *

import math

# ROS
import tf_transformations



def normalize(v):
  norm = np.linalg.norm(v)
  if norm < 0.00001:
    return v
  return v / norm



class Quaternion:

  @staticmethod
  def normalize(v):
    return normalize(v)

  @staticmethod
  def zerosQuat():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

  @staticmethod
  def zerosQuatSimp():
    return np.array([1.0, 0.0], dtype=float)

  @staticmethod
  def setQuatSimp(v):
    return Quaternion.normalize(v)

  @staticmethod
  def checkConsistencyQuat(v):
    tol=0.2
    norm = np.linalg.norm(v)
    if(abs(1.0-norm)>=tol):
      vout = Quaternion.zerosQuat()
    else:
      vout = Quaternion.normalize(v)
    return vout


  @staticmethod
  def quatFromRollPitchYaw(roll, pitch, yaw):
    # Closed-form roll-pitch-yaw (extrinsic 'sxyz') -> quaternion [w,x,y,z].
    # Equivalent to tf_transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
    # (verified to match it to float64 machine precision), without building/multiplying
    # rotation matrices for what is just a fixed formula.
    cr = math.cos(0.5*roll)
    sr = math.sin(0.5*roll)
    cp = math.cos(0.5*pitch)
    sp = math.sin(0.5*pitch)
    cy = math.cos(0.5*yaw)
    sy = math.sin(0.5*yaw)

    return np.array([
      cr*cp*cy + sr*sp*sy,
      sr*cp*cy - cr*sp*sy,
      cr*sp*cy + sr*cp*sy,
      cr*cp*sy - sr*sp*cy,
    ], dtype=float)


  @staticmethod
  def getSimplifiedQuatRobotAtti(robot_atti_quat):

    robot_atti_quat_tf = np.roll(robot_atti_quat, -1)
    robot_atti_ang = tf_transformations.euler_from_quaternion(robot_atti_quat_tf, axes='sxyz')
    robot_atti_ang_yaw = robot_atti_ang[2]

    robot_atti_hor_quat_tf = tf_transformations.quaternion_from_euler(0, 0, robot_atti_ang_yaw, axes='sxyz')
    robot_atti_hor_quat = np.roll(robot_atti_hor_quat_tf, 1)
    robot_atti_quat_simp = Quaternion.zerosQuatSimp()
    robot_atti_quat_simp[0] = robot_atti_hor_quat[0]
    robot_atti_quat_simp[1] = robot_atti_hor_quat[3]
    robot_atti_quat_simp = Quaternion.normalize(robot_atti_quat_simp)

    return robot_atti_quat_simp


  @staticmethod
  def quatProd(p, q):
    return np.array([
      p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
      p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
      p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
      p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0],
    ], dtype=float)

  @staticmethod
  def quatConj(p):
    return np.array([p[0], -p[1], -p[2], -p[3]], dtype=float)


  @staticmethod
  def quatSimpProd(q1, q2):
    return np.array([q1[0]*q2[0]-q1[1]*q2[1], q1[0]*q2[1]+q1[1]*q2[0]], dtype=float)


  @staticmethod
  def quatSimpConj(p):
    return np.array([p[0], -p[1]], dtype=float)


  @staticmethod
  def computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2):

    error_quat_simp = np.array([
      atti_quat_simp_1[0]*atti_quat_simp_2[0]+atti_quat_simp_1[1]*atti_quat_simp_2[1],
      atti_quat_simp_1[1]*atti_quat_simp_2[0]-atti_quat_simp_1[0]*atti_quat_simp_2[1],
    ], dtype=float)
    if(error_quat_simp[0] < 0):
      error_quat_simp = -1 * error_quat_simp

    return error_quat_simp


  @staticmethod
  def quatSimpFromAngle(angle):
    quatSimp = np.array([math.cos(0.5*angle), math.sin(0.5*angle)], dtype=float)

    if(quatSimp[0] < 0):
      quatSimp = -1 * quatSimp

    return quatSimp


  @staticmethod
  def angleFromQuatSimp(quatSimp):
    angle = 0.0

    if(quatSimp[0] < 0):
      quatSimp = -1 * quatSimp

    angle = 2.0 * math.atan(quatSimp[1]/quatSimp[0])

    return angle


  @staticmethod
  def angleDiffFromQuatSimp(atti_quat_simp_1, atti_quat_simp_2):

    return Quaternion.angleFromQuatSimp(Quaternion.computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2))


  @staticmethod
  def errorFromQuatSimp(quatSimp):
    error = 0.0

    if(quatSimp[0] < 0):
      quatSimp = -1 * quatSimp

    error = 2.0 * quatSimp[1].item()

    return error


  @staticmethod
  def errorDiffFromQuatSimp(atti_quat_simp_1, atti_quat_simp_2):

    return Quaternion.errorFromQuatSimp(Quaternion.computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2))


  @staticmethod
  def rotMat2dFromAngle(angle):
    rotMat2d = np.zeros((2,2), dtype=float)

    rotMat2d[0,0] = math.cos(angle)
    rotMat2d[0,1] = -math.sin(angle)
    rotMat2d[1,0] = math.sin(angle)
    rotMat2d[1,1] = math.cos(angle)

    return rotMat2d


  @staticmethod
  def rotMat3dFromAngle(angle):
    rotMat3d = np.zeros((3,3), dtype=float)

    rotMat3d[0:2, 0:2] = Quaternion.rotMat2dFromAngle(angle)
    rotMat3d[2,2] = 1.0

    return rotMat3d


  @staticmethod
  def rotMat2dFromQuatSimp(quatSimp):

    rotMat2d = Quaternion.rotMat2dFromAngle(Quaternion.angleFromQuatSimp(quatSimp))

    return rotMat2d


  @staticmethod
  def rotMat3dFromQuatSimp(quatSimp):

    rotMat3d = Quaternion.rotMat3dFromAngle(Quaternion.angleFromQuatSimp(quatSimp))

    return rotMat3d


  @staticmethod
  def diffRotMat3dWrtAngleFromAngle(angle):
    diffRotMat = np.zeros((3,3), dtype=float)

    diffRotMat[0,0] = -math.sin(angle)
    diffRotMat[0,1] = -math.cos(angle)
    diffRotMat[1,0] = math.cos(angle)
    diffRotMat[1,1] = -math.sin(angle)
    diffRotMat[2,2] = 0.0

    return diffRotMat



class Pose:

  parent_frame = ''
  child_frame = ''

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

    return Quaternion.computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2)


  @staticmethod
  def computeScalarDiffFromDiffQuatSimp(delta_atti_quat_simp):

    if(delta_atti_quat_simp[0] < 0):
      delta_atti_quat_simp = -1 * delta_atti_quat_simp

    error_att = 2.0 * delta_atti_quat_simp[1].item()

    return error_att


  @staticmethod
  def computeAngleDiffFromDiffQuatSimp(delta_atti_quat_simp):

    if(delta_atti_quat_simp[0] < 0):
      delta_atti_quat_simp = -1 * delta_atti_quat_simp

    error_att = 2.0 * math.atan(delta_atti_quat_simp[1]/delta_atti_quat_simp[0])

    return error_att


  @staticmethod
  def computePoseSimpDifference(posi_1, atti_quat_simp_1, posi_2, atti_quat_simp_2):

    # Position
    delta_posi = posi_1 - posi_2

    # Attitude
    delta_atti_quat_simp = PoseAlgebra.computeDiffQuatSimp(atti_quat_simp_1, atti_quat_simp_2)


    # End
    return delta_posi, delta_atti_quat_simp


  @staticmethod
  # pc = p1 + p2
  def computePoseSimpComposition(posi_1, atti_quat_simp_1, posi_2, atti_quat_simp_2):

    # Position
    posi_comp = np.matmul(Quaternion.rotMat3dFromQuatSimp(atti_quat_simp_1), posi_2) + posi_1

    # Attitude
    atti_quat_simp_comp = Quaternion.quatSimpProd(atti_quat_simp_1, atti_quat_simp_2)

    # End
    return posi_comp, atti_quat_simp_comp


  @staticmethod
  # pc = -p1
  def computePoseSimpInversion(posi_1, atti_quat_simp_1):

    # Position
    posi_inv = -np.matmul(np.transpose(Quaternion.rotMat3dFromQuatSimp(atti_quat_simp_1)), posi_1)

    # Attitude
    atti_quat_simp_inv = Quaternion.quatSimpConj(atti_quat_simp_1)

    # End
    return posi_inv, atti_quat_simp_inv


  @staticmethod
  # pc = p1 - p2
  def computePoseSimpPostSubstraction(posi_1, atti_quat_simp_1, posi_2, atti_quat_simp_2):

    #
    posi_2_inv, atti_quat_simp_2_inv = PoseAlgebra.computePoseSimpInversion(posi_2, atti_quat_simp_2)

    #
    posi_subs, atti_quat_simp_subs = PoseAlgebra.computePoseSimpComposition(posi_1, atti_quat_simp_1, posi_2_inv, atti_quat_simp_2_inv)

    # End
    return posi_subs, atti_quat_simp_subs


  @staticmethod
  # pc = - p1 + p2
  def computePoseSimpPreSubstraction(posi_1, atti_quat_simp_1, posi_2, atti_quat_simp_2):

    #
    posi_1_inv, atti_quat_simp_1_inv = PoseAlgebra.computePoseSimpInversion(posi_1, atti_quat_simp_1)

    #
    posi_subs, atti_quat_simp_subs = PoseAlgebra.computePoseSimpComposition(posi_1_inv, atti_quat_simp_1_inv, posi_2, atti_quat_simp_2)

    # End
    return posi_subs, atti_quat_simp_subs




class Conversions:

  @staticmethod
  def convertVelLinFromRobotToWorld(robot_velo_lin_robot, robot_atti_quat_in, flag_quat_simp=True):

    if(flag_quat_simp):
      # cos(yaw)/sin(yaw) from the half-angle quaternion via double-angle identities;
      # avoids a tf_transformations matrix round-trip for what is just a 2D yaw rotation
      cos_yaw = robot_atti_quat_in[0]*robot_atti_quat_in[0] - robot_atti_quat_in[1]*robot_atti_quat_in[1]
      sin_yaw = 2.0*robot_atti_quat_in[0]*robot_atti_quat_in[1]
    else:
      robot_atti_quat = Quaternion.normalize(robot_atti_quat_in)
      robot_atti_quat_tf = np.roll(robot_atti_quat, -1)
      robot_atti_ang = tf_transformations.euler_from_quaternion(robot_atti_quat_tf, axes='sxyz')
      robot_atti_ang_yaw = robot_atti_ang[2]
      cos_yaw = math.cos(robot_atti_ang_yaw)
      sin_yaw = math.sin(robot_atti_ang_yaw)

    return np.array([
      cos_yaw*robot_velo_lin_robot[0] - sin_yaw*robot_velo_lin_robot[1],
      sin_yaw*robot_velo_lin_robot[0] + cos_yaw*robot_velo_lin_robot[1],
      robot_velo_lin_robot[2],
    ], dtype=float)

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
    robot_atti_ang = tf_transformations.euler_from_quaternion(robot_atti_quat_tf, axes='sxyz')
    robot_atti_ang_yaw = robot_atti_ang[2]

    robot_velo_lin_robot = np.zeros((3,), dtype=float)

    robot_velo_lin_robot[0] = math.cos(robot_atti_ang_yaw)*robot_velo_lin_world[0]+math.sin(robot_atti_ang_yaw)*robot_velo_lin_world[1]
    robot_velo_lin_robot[1] = -math.sin(robot_atti_ang_yaw)*robot_velo_lin_world[0]+math.cos(robot_atti_ang_yaw)*robot_velo_lin_world[1]
    robot_velo_lin_robot[2] = robot_velo_lin_world[2]

    return robot_velo_lin_robot

  @staticmethod
  def convertVelAngFromWorldToRobot(robot_velo_ang_world, robot_atti_quat_in, flag_quat_simp=True):

    return robot_velo_ang_world



class Circle3D:

  id_lab = None
  position = None
  attitude_quat_simp = None
  parent_frame = None
  circle_radius = None


  def __init__(self):

    self.id_lab = -1
    self.position = np.zeros((3,), dtype=float)
    self.attitude_quat_simp = Quaternion.zerosQuatSimp()
    self.parent_frame = ''
    self.circle_radius = 0.0

    return






def isPointInCircle(point, circle_center, circle_radius):

  circle_impl_equ = (point[0]-circle_center[0])**2 + (point[1]-circle_center[1])**2 - circle_radius**2

  if(circle_impl_equ < 1.0):
    return True
  else:
    return False


def distancePointCircle(point_2d, circle_center_2d, circle_radius):

  distance_circle = 0.0

  distance_center = np.linalg.norm(circle_center_2d-point_2d)

  if(distance_center <= circle_radius):
    distance_circle = 0.0
  else:
    distance_circle = distance_center - circle_radius

  return distance_circle


def distanceSegmentCircle(point_2d_segment_1, point_2d_segment_2, circle_center_2d, circle_radius):

  # LINE

  v_s1 = np.zeros((2,), dtype=float)
  v_s1 = point_2d_segment_2 - point_2d_segment_1
  v_s1 = normalize(v_s1)

  # The two waypoints are the same!
  if(np.linalg.norm(v_s1) < 0.00001):
    return distancePointCircle(point_2d_segment_1, circle_center_2d, circle_radius)

  v_s2 = np.zeros((2,), dtype=float)
  v_s2[0] = v_s1[1]
  v_s2[1] = -v_s1[0]

  
  mat_int_s1_s2 = np.array([v_s1, -v_s2]).T

  sol_int_s1_s2 = np.matmul(np.linalg.inv(mat_int_s1_s2), (circle_center_2d - point_2d_segment_1))

  point_int_l = sol_int_s1_s2[1] * v_s2 + circle_center_2d

  dist_pi_c = abs(sol_int_s1_s2[1].item())


  # SEGMENT

  dist_p1_pi = np.linalg.norm(point_2d_segment_1-point_int_l)

  dist_p1_p2 = np.linalg.norm(point_2d_segment_1-point_2d_segment_2)

  dist_p2_pi = np.linalg.norm(point_2d_segment_2-point_int_l)

  distance_circle = 0.0

  if(dist_p1_pi<=dist_p1_p2 and dist_p2_pi<=dist_p1_p2):

    # Case easy

    if(dist_pi_c<circle_radius):
      distance_circle = 0.0
    elif(dist_pi_c==circle_radius):
      distance_circle = 0.0
    else:
      distance_circle = dist_pi_c - circle_radius


  else:

    dist_p1_c = np.linalg.norm(point_2d_segment_1-circle_center_2d)

    if(dist_p1_c<=circle_radius):
      dist_p1_circ = 0.0
    else:
      dist_p1_circ = dist_p1_c-circle_radius

    dist_p2_c = np.linalg.norm(point_2d_segment_2-circle_center_2d)

    if(dist_p2_c<=circle_radius):
      dist_p2_circ = 0.0
    else:
      dist_p2_circ = dist_p2_c-circle_radius

    distance_circle = min(dist_p1_circ, dist_p2_circ)


  return distance_circle



def pointOverSegment(point, point_segment_1, point_segment_2):


  flag_degradated = False
  point_over_segment = np.zeros((3,), dtype=float)


  v_s1 = np.zeros((3,), dtype=float)
  v_s1 = point_segment_2 - point_segment_1
  v_s1 = normalize(v_s1)

  v_1_0 = point - point_segment_1


  dot_prod = np.dot(v_s1, v_1_0)


  point_over_segment = dot_prod * v_s1 + point_segment_1


  dist_p1_ps = np.linalg.norm(point_over_segment - point_segment_1)
  dist_p2_ps = np.linalg.norm(point_over_segment - point_segment_2)
  dist_p1_p2 = np.linalg.norm(point_segment_2 - point_segment_1)


  if(dist_p1_ps<=dist_p1_p2 and dist_p2_ps<=dist_p1_p2):

    pass

  else:

    flag_degradated = True

    if(dist_p1_ps < dist_p2_ps):
      point_over_segment = point_segment_1
    else:
      point_over_segment = point_segment_2


  return point_over_segment, flag_degradated
