import rospy
import baxter_interface as bax
import baxter_pykdl as kdl
from math import pi, sqrt
import numpy as np

from drum_surfaces import drum_poses
from transform_utils import round_angles, unitvec, mat2quat
from copy import deepcopy

rospy.init_node('ik_mover')

l, r = 'left', 'right'

lkin = kdl.baxter_kinematics(l)
rkin = kdl.baxter_kinematics(r)

kin = {l: lkin, r:rkin}

left = bax.Limb(l)
right = bax.Limb(r)

CARTESIAN_TOL = 0.1

def cur_point(limb, correct=True):
    point= bax.Limb(limb).endpoint_pose()['position']
    x,y,z = point.x, point.y, point.z
    if correct:
        z += 0.93
    return [x, y, z]

def point_distance(a, b):
    return sqrt(((a[0]-b[0])**2)+((a[1]-b[1])**2)+((a[2]-b[2])**2))

def goto_point(target_point, limb):
    # correct for robot offset in simulation:
    corrected_point = target_point[0], target_point[1], target_point[2]+0.93
    assert limb in [l, r]
    arm = bax.Limb(limb)
    solution = kin[limb].inverse_kinematics(corrected_point)
    if solution is not None:
        print('Found solution:\n%s' % str(solution))
        solution = round_angles(solution)
        print('Rounded to:\n%s' % solution)
        print('Current angles:\n%s' % [arm.joint_angles()[j] for j in arm.joint_names()])
        sol_dict = {}
        names = arm.joint_names()
        for j in range(len(sol_dict)):
            sol_dict[names[j]] = solution[j]
        rate = rospy.Rate(50)
        done = False
        print('Moving to target position...')
        while not done:
            print(arm.set_joint_positions(sol_dict))

            # print('cur point: %s' % str(cur_point(limb)))
            # print('target point: %s' % str(target_point))
            error = point_distance(cur_point(limb), target_point)
            # print('error: %s' % error)
            if abs(error) < CARTESIAN_TOL:
                done = True 
            rate.sleep()
        print('Done moving.')
    else:
        raise Exception('No valid solution found \nfor the point: x:%.2f y:%.2f z:%.2f' % (target_point[0],
        target_point[1], 
        target_point[2]))

def dict_angles(angles, limb):
    names = bax.Limb(limb).joint_names()
    d = {}
    for j in range(len(names)):
        d[names[j]] = angles[j]
    return d

def list_angles(angles, limb):
    names = bax.Limb(limb).joint_names()
    dict_angles = bax.Limb(limb).joint_angles()
    angles = [dict_angles[n] for n in names]
    return angles

def orient_towards_point(point):
    """takes a point in cartesian space relative to origin as iterable,
    and finds the rotation matrix with basis vector z
    that points along the vector of that point"""
    # the new basis vector z points along the position vector:
    basis_z = unitvec(point)

    # choose an arbitrary perpendicular basis vector x
    # by taking cross product of z with an arbitrary vector:
    basis_x = np.cross(basis_z, np.asarray([1,0,0]))

    if np.sum(basis_x) == 0:
        # if we happened to accidentally choose a parallel vector, try another:
        basis_x = np.cross(basis_z, np.asarray([0,1,0]))
    # scale x to be a unit vector:
    basis_x = unitvec(basis_x)

    # final basis vector is now determined:
    basis_y = np.cross(basis_z, basis_x)
    rot = np.column_stack([basis_x, basis_y, basis_z])

    # ensure right handed coordinate frame:
    if np.linalg.det(rot) < 0:
        rot[:,1] *= -1

    # sanity check:
    assert np.linalg.det(rot) == 1.0
    return rot



# test everything:

print('\n\n')
target_pos = drum_poses['cymbal2'][:3]
target_ori = orient_towards_point(target_pos)
target_quat = mat2quat(target_ori)

try:
    cur_sol = round_angles(lkin.inverse_kinematics(target_pos, target_quat))
    print(cur_sol)
    dsol = dict_angles(cur_sol, l)
    fwd = lkin.forward_position_kinematics(dsol)[:3]
    print(fwd)
except:
    print('No solution found')