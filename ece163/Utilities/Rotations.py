"""
Author: Kevin Jesubalan (kjesubal@ucsc.edu)
File contains methods which helps with rotations using rotation matrices
"""
import math
from . import MatrixMath


def euler2DCM(yaw, pitch, roll):
    """ Create the direction cosine matrix, R, from the euler angles (assumed to be in radians) """
    rvtov1 = [[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]  # yaw rotation
    rv1tov2 = [[math.cos(pitch), 0, -math.sin(pitch)], [0, 1, 0],
               [math.sin(pitch), 0, math.cos(pitch)]]  # pitch rotation
    rv2tob = [[1, 0, 0], [0, math.cos(roll), math.sin(roll)], [0, -math.sin(roll), math.cos(roll)]]  # roll rotation
    rvtob = MatrixMath.multiply(rv2tob, MatrixMath.multiply(rv1tov2, rvtov1))
    return rvtob


def dcm2Euler(DCM):
    """Extracts the Euler angles from the rotation matrix, in the form of yaw, pitch, roll"""
    pitch_check = DCM[0][2]  # checking to make sure range is within [-1,1]

    if pitch_check > 1:
        pitch_check = 1
    elif pitch_check < -1:
        pitch_check = -1

    pitch = -1 * math.asin(pitch_check)
    yaw = math.atan2(DCM[0][1], DCM[0][0])
    roll = math.atan2(DCM[1][2], DCM[2][2])

    return yaw, pitch, roll


def ned2enu(points):
    rotation_matrix = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    return MatrixMath.multiply(points, rotation_matrix)
