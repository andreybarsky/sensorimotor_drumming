#!/usr/bin/python2

import numpy as np
import math
import rospy
import baxter_interface as bax
from poser import load_pose, move_to_pose
import rospkg
from drum_surfaces import drum_poses, drum_names

# ordered joint angle names, global variable:
LNAMES = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
RNAMES = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

pkg_path = rospkg.RosPack().get_path('drumming')

pose_dir = pkg_path + '/motion/waypoints/'

def pose_to_array(pose_dict):
    """takes a dict of joint angles and converts
    to np array ordered by joint number"""
    # surmise the arm we are using:
    names = LNAMES if 'left' in pose_dict.keys()[0] else RNAMES
    pose_array = np.zeros(len(names))
    for n in range(len(names)):
        pose_array[n] = pose_dict[names[n]]
    return pose_array

def interpolate(pose_arr1, pose_arr2, steps=10):
    """takes a pair of poses for one arm
    and outputs a trajectory that linearly
    interpolates between them, in x steps"""
    num_joints = len(pose_arr1)
    traj = np.zeros((steps, num_joints))
    poses = np.row_stack((pose_arr1, pose_arr2))
    # a vector of pose weights for linear interpolation:
    # (starting from pose 1 and gradually becoming pose 2)
    weighting = np.linspace(1,0,steps)
    # take weighted average of the two poses at each time step:
    for t in range(steps):
        arr1_weight = weighting[t]
        traj[t,:] = np.average(poses, axis=0, weights=[weighting[t], 1-weighting[t]])
    return traj

def poses_to_traj(poses, steps=10):
    pose_arrays = [pose_to_array(p) for p in poses]
    pose_pairs = [(pose_arrays[x], pose_arrays[x+1]) for x in range(len(poses)-1)]
    movements = [interpolate(pp[0], pp[1], steps=steps) for pp in pose_pairs]
    traj = np.row_stack(movements)
    return traj

def arr_to_pose(arr_line, arm):
    """takes a row of an np.array and translates to pose dict for the chosen arm"""
    assert arm in ['left', 'right']
    names = LNAMES if arm is 'left' else RNAMES
    pose = {}
    for n in range(len(names)):
        pose[names[n]] = arr_line[n]
    return pose

def play_arm_traj(traj, arm, tstep=0.1):
    """takes an array of joint angles, an arm and a time step in seconds
    and goes to those joint angles in that time for that arm"""
    rate = rospy.Rate(1.0 / tstep)
    arm_obj = bax.Limb(arm)
    num_steps = traj.shape[0]
    print('Moving...')
    for t in range(num_steps):
        pose = arr_to_pose(traj[t,:], arm)
        arm_obj.set_joint_positions(pose)
        rate.sleep()
    print('Done moving')

def play_full_traj(left_traj, right_traj, tstep=0.1):
    """as play_arm_traj above, but for both arms simultaneously"""
    rate = rospy.Rate(1.0 / tstep)
    left, right = bax.Limb('left'), bax.Limb('right')
    num_steps = left_traj.shape[0]
    assert left_traj.shape == right_traj.shape
    print('Moving...')
    for t in range(num_steps):
        lpose, rpose = arr_to_pose(left_traj[t,:], 'left'), arr_to_pose(right_traj[t,:], 'right')
        left.set_joint_positions(lpose)
        right.set_joint_positions(rpose)
        rate.sleep()
    print ('Done moving')

class DrumPattern(object):
    """a DrumPattern is a named drum, and a series of joint waypoints 
        required to hit that drum: a rest, strike, rest triplet.
    the class method trajectory() returns the trajectory of joint angles
        required for a given sampling rate and a given target """ 
    def __init__(self, name, arm, waypoints):
        self.name = name # name of the drum
        self.arm = arm # which arm is drumming, one of 'left', 'right'
        self.waypoints = waypoints # list of arrays of joint angles, [start, strike, end]

    def trajectory(self, start_frame, impact_frame, end_frame):
        """given a start, impact and return time in seconds, and a rate in hz,
        return the trajectory of joint angles satisfying those times"""
        # num_frames = int(np.ceil(rate * end_time))
        # impact_frame = int(np.ceil(rate * impact_time))

        # offset all frames to begin from zero:
        end_frame -= start_frame
        impact_frame -= start_frame
        start_frame = 0
        
        start = pose_to_array(self.waypoints[0])
        strike = pose_to_array(self.waypoints[1])
        end = pose_to_array(self.waypoints[0])

        traj = np.zeros((end_frame, 7))
        traj[start_frame:impact_frame,:] = interpolate(start, strike, steps= impact_frame - start_frame)
        traj[impact_frame:,:] = interpolate(strike, end, steps = end_frame - impact_frame)

        return traj

    def __str__(self):
        return self.name

    def __getitem__(self, i):
        return self.waypoints[i]

# the arm each drum is played with:
drum_arms ={'hihat'  : 'left' ,
            'cymbal1': 'left' ,
            'tom1'   : 'left' ,
            'snare'  : 'right',
            'cymbal2': 'right',
            'tom2'   : 'right'}


# dict of drum names -> filenames of waypoints to hit those drums
# first file is the rest position, second file is the impact position
drum_waypoint_files = {
    'hihat'  : [pose_dir+'l_hihat_ready.csv',  pose_dir+'l_hihat_hit.csv'],
    'cymbal1': [pose_dir+'l_cymbal_ready.csv', pose_dir+'l_cymbal_hit.csv'],
    'tom1'   : [pose_dir+'l_tom_ready.csv',    pose_dir+'l_tom_hit.csv'],
    'snare'  : [pose_dir+'r_snare_ready.csv',  pose_dir+'r_snare_hit.csv'],
    'cymbal2': [pose_dir+'r_cymbal_ready.csv', pose_dir+'r_cymbal_hit.csv'],
    'tom2'   : [pose_dir+'r_tom_ready.csv',    pose_dir+'r_tom_hit.csv']
}

# load those waypoints as arrays into a new dict:
drum_waypoints = {}
for key, val in drum_waypoint_files.iteritems():
    arm_idx = 0 if drum_arms[key] is 'left' else 1
    # assign (rest, hit) tuple as new key, but only for the arm that waypoint belongs to:
    drum_waypoints[key] = load_pose(val[0])[arm_idx], load_pose(val[1])[arm_idx]

# and convert into a dict of DrumPattern objects:
drum_patterns = {}
for name, waypoints in drum_waypoints.iteritems():
    drum_patterns[name] = DrumPattern(name, drum_arms[name], [waypoints[0], waypoints[1]])

class DrumBeater(object):
    def __init__(self, drum_arms, drum_patterns, strike_time, framerate=50):
        self.drum_arms = drum_arms
        self.drum_patterns = drum_patterns
        self.drum_names = drum_arms.keys()
        self.strike_time = strike_time # time allowed in seconds for each discrete down/up drumming motion
        
        # constants for interpolating between the starts and ends of beats:
        self.min_move_time = 0.2 # must allow this much time to move between ready positions
        self.max_move_time = 0.5 # moving between ready positions will never take longer than this
                                     # (instead we will idle until we must move)

        self.framerate = framerate # arm update rate in hz

    def beats_to_traj(self, beat_dict, duration=None, framerate=None):
        """Takes a labelled dict of beat lists: {'drum': [0.1, 0.2, etc.]}
        and outputs the joint angles for each arm at each frame of time.

        args:
        beat_dict:: a dict where keys are drum names, and values are
            the time in seconds when that drum should be struck.
            values must be monotonically ascending ordered.
        framerate:: the rate in hz at which arm angles are updated"""

        if framerate is None:
            framerate = self.framerate

        if duration is None:
            # find the absolute latest beat, and allow 0.1 sec to return to rest:
            duration = np.max([np.max(b) for b in beat_dict.values()]) + self.strike_time/2.
        
        num_frames = int(np.floor(duration * framerate))

        beat_dict_left = {}
        beat_dict_right = {}
        for name, beats in beat_dict.iteritems():
            if self.drum_arms[name] == 'left':
                beat_dict_left[name] = beats
            elif self.drum_arms[name] == 'right':
                beat_dict_right[name] = beats

        trajectories = []

        for bd in (beat_dict_left, beat_dict_right):
            if len(bd) == 0:
                trajectories.append(None)
            else:
                names, beats = self.beat_dict_to_lists(bd)
                arm_traj = np.zeros((num_frames, 7))
                num_beats = len(beats)

                arm_traj.fill(np.nan) # fill with non-numeric values to catch errors if we fail to fill anything in

                last_beat_frame = 0

                for b in range(num_beats):
                    drum = names[b]
                    impact_time = beats[b]
                    impact_frame = int(math.floor(impact_time * framerate))

                    start_time = impact_time - self.strike_time/2.
                    start_frame = int(math.floor(start_time * framerate))

                    end_time = impact_time + self.strike_time/2.
                    end_frame = int(math.floor(end_time * framerate))

                    beat_traj = self.drum_patterns[drum].trajectory(start_frame, impact_frame, end_frame)
                    # print('beat_traj shape before clipping: %s' % str(beat_traj.shape))
                    # sometimes we'll need to start moving before the start of the song, so let's handle this here:
                    if start_frame < 0:
                        extra_frames = -start_frame
                        start_frame = 0
                        # slice off the start of the beat trajectory:
                        beat_traj = beat_traj[extra_frames:,:]
                    elif b == 0:
                        # if this is the first beat and the start frame is after 0, we need to backfill to start:
                        # print('backfilling up to %d' % start_frame)
                        arm_traj[:start_frame,:] = beat_traj[0,:]
                    else:
                        # interpolate between the end of the last movement and the start of this one

                        # last frame of last movement:
                        last_position = arm_traj[last_beat_frame,:]
                        # first frame of this movement:
                        new_position = pose_to_array(self.drum_patterns[drum].waypoints[0])
                        # duration of movement:

                        interval_frames = start_frame - last_beat_frame
                        min_frames = int(math.floor(self.min_move_time * framerate))
                        max_frames = int(math.floor(self.max_move_time * framerate))

                        if interval_frames < min_frames:
                            raise Exception('Not enough time to move for %dth beat. Need: %.3f secs, Have: %.3f secs' % (b, self.min_move_time, (interval_frames * 1.0) / framerate))

                        movement_frames = np.min((max_frames, interval_frames))

                        idle_frames = interval_frames - movement_frames

                        # stay still while idling:
                        arm_traj[last_beat_frame:last_beat_frame+idle_frames] = arm_traj[last_beat_frame,:]

                        # then move:

                        ready_traj = interpolate(last_position, new_position, movement_frames)
                        # print('filling between frames %d and %d' % (last_beat_frame, start_frame))
                        # print('last position is:')
                        # print(last_position)
                        # print('new position is:')
                        # print new_position

                        arm_traj[last_beat_frame+idle_frames:start_frame,:] = ready_traj

                    # print('drum strike trajectory between frames %d and %d' % (start_frame, end_frame))
                    # print ('start frame: %d' % start_frame)
                    # print ('end frame: %d' % end_frame)
                    # print ('beat traj shape: %s' % str(beat_traj.shape))
                    # print('beat traj:\n%s' % str(beat_traj))
                    arm_traj[start_frame:end_frame,:] = beat_traj

                    #
                    last_beat_frame = end_frame - 1

                arm_traj[last_beat_frame:,:] = arm_traj[last_beat_frame,:]
                trajectories.append(arm_traj)
        return trajectories[0], trajectories[1]


    def beat_dict_to_lists(self, beat_dict):
        """takes a labelled dict of beat lists, and outputs two ordered lists:
        one of beat times, one of the drums of those beats"""
        unordered_names = []
        unordered_times = []
        for name, times in beat_dict.iteritems():
            for t in times:
                unordered_names.append(name)
                unordered_times.append(t)

        # now we find the beat order and sort both arrays (decorate/sort/undecorate):
        ordered_times, ordered_names = (list(t) for t in zip(*sorted(zip(unordered_times, unordered_names))))
        return ordered_names, ordered_times

    def required_vel(self, traj, times):
        dx = np.diff(traj, axis=0)
        dt = np.diff(times, axis=0)
        velocities = dx / dt
        joint_max_vels = np.max(velocities, axis=0)
        return np.max(joint_max_vels)

    def required_accel(self, traj, times):
        dx = np.diff(traj, axis=0)
        dx2 = np.diff(dx, axis=0)
        dt = np.diff(times, axis=0)[1:]
        accels = dx2 / dt
        joint_max_accels = np.max(accels, axis=0)
        return np.max(joint_max_accels)




if __name__ == '__main__':
    rospy.init_node('traj')

    bax.RobotEnable(True).enable()

    example_beats = {'snare': np.arange(0,10, 2.), 'cymbal2': np.arange(1,11, 2.),
                'hihat': np.arange(0,11,1.)}
    easy_beats = {'snare': np.arange(1, 8, 1.2),
                  'cymbal2': np.arange(1.6, 8.6, 1.2)}

    beater = DrumBeater(drum_arms, drum_patterns, strike_time = 0.2)
    left_traj, right_traj = beater.beats_to_traj(example_beats)

    # import matplotlib.pyplot as plt
    # #e0 = plt.plot(left_traj[:,2])
    # e1 = plt.plot(right_traj[:,3])

    print('Moving to start position...')
    move_to_pose(arr_to_pose(left_traj[0,:], 'left'), arr_to_pose(right_traj[0,:], 'right'))

    play_full_traj(left_traj, right_traj, 1./50)