import numpy as np
from math import sqrt, pi, atan2, cos as c, sin as s # for compactness
from geometry_msgs.msg import Pose, Point
from tf.transformations import euler_from_quaternion as q2e, quaternion_from_euler as e2q

def transform_matrix(px,py,pz,r,p,y):
    """given pose coordinates px,py,pz and euler angles
    for roll, pitch, yaw, return the 4*4 homogenous
    matrix representing that transformation from base"""

    # this is just the matrix representing translation by px, py, pz
    # then rotation around z, y, x in that order (yaw, pitch, roll)
    transform = np.asarray(
        [[c(y)*c(p), c(y)*s(p)*s(r)-s(y)*c(r), c(y)*s(p)*c(r)+s(y)*s(r), px],
         [s(y)*c(p), s(y)*s(p)*s(r)+c(y)*c(r), s(y)*s(p)*c(r)-c(y)*s(r), py],
         [-s(p),     c(p)*s(r),                c(p)*c(r),                pz],
         [0,         0,                        0,                        1]])
        # aren't we glad we made it compact?
    return transform

def invert_transform(transform):
    """given a 4*4 homogenous matrix with a rotation part
    and a translation part, return the matrix that performs 
    the inverse operation"""
    rot_part = transform[:3,:3]
    trans_part = transform[:3,3]

    # inverse of a rotation is just its transpose:
    inverse_rot = rot_part.T
    # inverse of a translation depends on the rotation:
    inverse_trans = -np.dot(inverse_rot, trans_part)

    # insert into 4*4 homogenous matrix:
    inverse = np.eye(4)
    inverse[:3,:3] = inverse_rot
    inverse[:3,3] = inverse_trans
    return inverse

def round_angles(angles):
    """given a list of joint angles, truncate them into the range -pi:pi"""
    for a in range(len(angles)):
        while angles[a] > pi or angles[a] < -pi:
            if angles[a] > 0:
                angles[a] -= (2*pi)
            elif angles[a] < 0:
                angles[a] += (2*pi)
    return angles

def deg2rad(deg):
    return deg * (pi / 180)

def rad2deg(rad):
    return rad / (pi / 180)

def unitvec(vec):
    """takes a vector in arbitrary dimensional space
    and returns the unit vector with the same orientation"""
    length = np.linalg.norm(vec)
    return vec / length

def mat2quat(m):
    """convert rotation matrix to its quaternion representation"""
    qw = sqrt(1.0 + m[0,0] + m[1,1] + m[2,2]) / 2.0
    qx = (m[2,1] - m[1,2]) / ( 4.0 * qw)
    qy = (m[0,2] - m[2,0]) / ( 4.0 * qw)
    qz = (m[1,0] - m[0,1]) / ( 4.0 * qw)
    return [qx, qy, qz, qw]

    # # placeholder java code if that doesn't work:
    # float tr = m00 + m11 + m22

    # if (tr > 0) { 
    #   float S = sqrt(tr+1.0) * 2; // S=4*qw 
    #   qw = 0.25 * S;
    #   qx = (m21 - m12) / S;
    #   qy = (m02 - m20) / S; 
    #   qz = (m10 - m01) / S; 
    # } else if ((m00 > m11)&(m00 > m22)) { 
    #   float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
    #   qw = (m21 - m12) / S;
    #   qx = 0.25 * S;
    #   qy = (m01 + m10) / S; 
    #   qz = (m02 + m20) / S; 
    # } else if (m11 > m22) { 
    #   float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
    #   qw = (m02 - m20) / S;
    #   qx = (m01 + m10) / S; 
    #   qy = 0.25 * S;
    #   qz = (m12 + m21) / S; 
    # } else { 
    #   float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
    #   qw = (m10 - m01) / S;
    #   qx = (m02 + m20) / S;
    #   qy = (m12 + m21) / S;
    #   qz = 0.25 * S;
    # }

def mat2rpy(m):
    sy = sqrt(m[0,0] * m[0,0] +  m[1,0] * m[1,0])    
    singular = sy < 1e-6
    if  not singular :
        x = atan2(m[2,1] , m[2,2])
        y = atan2(-m[2,0], sy)
        z = atan2(m[1,0], m[0,0])
    else :
        x = atan2(-m[1,2], m[1,1])
        y = atan2(-m[2,0], sy)
        z = 0
    return np.array([x, y, z])

def pose_to_matrix(pose):
    """takes ros pose message and converts to 4x4 homogenous matrix"""
    pos = [pose.position.x, pose.position.y, pose.position.z]
    quat = pose.orientation
    rpy = list(q2e((quat.x, quat.y, quat.z, quat.w)))
    mat = transform_matrix(*pos+rpy)
    return mat

def extract(mat, rot_type='rpy'):
    """takes homogenous 4x4 matrix, returns translation and rotation parts.
    rot can be 'rpy', 'quat' or 'matrix' for different representations"""
    position = mat[:3,3]
    rotation = mat[:3,:3]
    if 'rpy' in rot_type:
        orientation = mat2rpy(rotation)
        return position, orientation
    elif 'quat' in rot_type:
        orientation = mat2rpy(rotation)
        quat = e2q(*orientation)
        return position, quat
    elif 'mat' in rot_type:
        return position, orientation