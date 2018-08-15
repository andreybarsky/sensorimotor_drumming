import baxter_interface as bax
from baxter_interface import CHECK_VERSION
import rospy
import rospkg
import argparse
import numpy as np

# constants:
MOVE_RATE = 20.0 # hz
ANGLE_TOL = 0.1 # radians; to determine when we have reached the goal
STUCK_TOL = 0.05 # radians; to determine when arms are stuck and not moving anymore

LNAMES = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
RNAMES = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

pkg_path = rospkg.RosPack().get_path('drumming')

filedir = pkg_path + '/motion/waypoints/'


def check_goal(langles, rangles):
    """checks current joint states against desired inputs and returns boolean based on tolerance parameter"""
    left = bax.Limb('left')
    right = bax.Limb('right')

    cur_langles = []
    cur_rangles = []

    goal_langles = []
    goal_rangles = []

    # collect current left and right angles in order of jointnames
    for j in LNAMES: # make sure we capture them in correct order of jointnames
        cur_langles.append(left.joint_angles()[j])
        goal_langles.append(langles[j])
    for j in RNAMES:
        cur_rangles.append(right.joint_angles()[j])
        goal_rangles.append(rangles[j])

    goal_angles = goal_langles + goal_rangles
    cur_angles = cur_langles + cur_rangles

    differences = [abs(goal_angles[x] - cur_angles[x]) for x in range(len(goal_angles))]
    max_difference = np.max(differences)

    if max_difference > ANGLE_TOL:
        return False
    else:
        return True

def check_stuck(old_langles, old_rangles):
    left = bax.Limb('left')
    right = bax.Limb('right')
    
    # LNAMES = left.joint_names()
    # RNAMES = right.joint_names()

    cur_langles = left.joint_angles()
    cur_rangles = right.joint_angles()

    dof = len(LNAMES) + len(RNAMES)
    dof_each = dof / 2

    diffs = np.zeros(dof)
    for i in range(len(LNAMES)):
        diffs[i] = abs(cur_langles[LNAMES[i]] - old_langles[LNAMES[i]])
        diffs[i+dof_each] = abs(cur_rangles[RNAMES[i]] - old_rangles[RNAMES[i]])

    if np.max(diffs) > STUCK_TOL:
        return False
    else:
        return True


def move_to_pose(langles, rangles):
    # rospy.init_node('poser')
    # rospy.sleep(0.5) # let subscribers latch

    rs = bax.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    # we don't want to disable the robot when this is done - we want the arms to lock
    # def clean_shutdown():
    #    print("\nExiting example...")
    #    if not init_state:
    #        print("Disabling robot...")
    #        rs.disable()
    # rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    left = bax.Limb('left')
    right = bax.Limb('right')

    reached_goal = False
    rate = rospy.Rate(MOVE_RATE)

    old_langles, old_rangles = left.joint_angles(), right.joint_angles()

    print('Moving...')
    abort = False
    stuck_count = 0 # increment by 1 if we hit a stuck frame
    while not abort:
        left.set_joint_positions(langles) # left only for now
        right.set_joint_positions(rangles)
        if check_goal(langles, rangles):
            abort = True
        
        if check_stuck(old_langles, old_rangles):
            stuck_count += 1
        else:
            stuck_count = 0

        if stuck_count > 3:
            print('Arms stuck, aborting')
            abort = True

        old_langles, old_rangles = left.joint_angles(), right.joint_angles()
        rate.sleep()
    print('Finished moving.')

def mirror_pose(angles):
    """flip set of joint angles to be laterally mirror symmetric"""

    # determine which arm is given by the dict keys in the argument
    if 'left' in angles.keys()[0]:
        source = 'left'
    else:
        source = 'right'

    # LNAMES = bax.Limb('left').joint_names()
    # RNAMES = bax.Limb('right').joint_names()

    # we have to flip every other joint:
    flip = [-1,  1, -1,  1, -1,  1, -1]

    mirrored_angles = {}

    # swap the joint name to the other arm and flip if needed:
    if source == 'left':
        for n in range(len(LNAMES)): 
            mirrored_angles[RNAMES[n]] = angles[LNAMES[n]] * flip[n]
    elif source == 'right':
        for n in range(len(RNAMES)):
            mirrored_angles[LNAMES[n]] = angles[RNAMES[n]] * flip[n]

    return mirrored_angles
        

def load_pose(filename, split_arms = True):
    """loads a set of joint angles from csv.
    pose can be loaded as one 14-dof dict or two left/right 7-dof dicts"""
    # ljoints = bax.Limb('left').joint_names()
    # rjoints = bax.Limb('right').joint_names()
    with open(filename, 'r') as file:
        csv = file.read()
    angles = [float(x) for x in csv.split(',')]
    names = LNAMES + RNAMES
    joint_dict = {}
    for i in range(len(names)):
        joint_dict[names[i]] = angles[i]
    # this is messy I know:
    if split_arms is False:
        return joint_dict
    else:
        ldict = {}
        for j in LNAMES:
            ldict[j] = joint_dict[j]
        rdict = {}
        for j in RNAMES:
            rdict[j] = joint_dict[j]
        return ldict, rdict
        

def save_pose(joint_dict, filename):
    """saves a dict of joint angles as csv.
    joint_dict can be a dict of 14 angles, or a tuple of (left,right) 7-angle dicts"""
    # ljoints = bax.Limb('left').joint_names()
    # rjoints = bax.Limb('right').joint_names()
    if len(joint_dict) == 2: # if given a tuple of left/right joint angles, combine them
        dict1, dict2 = joint_dict
        joint_dict = {}
        for key, val in dict1.iteritems():
            joint_dict[key] = val
        for key, val in dict2.iteritems():
            joint_dict[key] = val
    jointstr = [str(joint_dict[n]) for n in LNAMES] + [str(joint_dict[n]) for n in RNAMES]
    csv = ','.join(jointstr)
    with open(filename, 'w') as file:
        file.write(csv)

def save_current_pose(filename):
    langles = bax.Limb('left').joint_angles()
    rangles = bax.Limb('right').joint_angles()
    save_pose((langles, rangles), filename)


def main():
    rospy.init_node('poser')
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '-pose', required=True, type=str, help='pose to assume - a csv file of joint angles')
    try:
        args = parser.parse_args()
        pose_file = args.p

        lpose, rpose = load_pose(pose_file, split_arms=True)

        # first move to neutral position with arms back:
        move_to_pose(lpose, rpose)
    except:
        print('No pose argument set.')

    


if __name__=='__main__':
    main()