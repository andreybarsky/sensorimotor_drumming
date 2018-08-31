import rospy
import numpy as np
import os, sys
import time
import pickle
from mmd_recorder import Recorder
from baxter_mmd import MultimodalData, load_mmd_obj
from trajectory_combiner import DrumBeater, arr_to_pose, move_to_pose, drum_arms, drum_patterns
import baxter_interface as bax

move_to_start=True
wait=False

# constants:
HZ = 50
IMG_DIM = [120, 120, 3]

datadir = 'trials/'


class PlaybackRecorder(object):
    def __init__(self, img_dims = [50,50,3], strike_time = 0.2):

        self.recorder = Recorder(camera_topic='/cameras/head_camera/image', 
                    collision_topic='drum_collision',
                    img_dim=[800,800,3], 
                    downsampled_img_dim=img_dims)
        self.strike_time = strike_time
        self.beater = DrumBeater(drum_arms, drum_patterns, self.strike_time)

    def move_to_start(self, left_traj, right_traj):
        """move to the starting position of a given trajectory"""
        move_to_pose(arr_to_pose(left_traj[0,:], 'left'), arr_to_pose(right_traj[0,:], 'right'))        

    def play_and_record(self, beats, filename, secs, hz=HZ, move_to_start=True, prompt=True):
        """play a trajectory for both arms and concurrently record multimodal data"""

        rostic = rospy.get_time()
        tic = time.time()
        
        # prepare mover:
        print('Processing required trajectory...')
        left_traj, right_traj = self.beater.beats_to_traj(beats, duration=secs, framerate=hz)
        rate = rospy.Rate(hz)
        left, right = bax.Limb('left'), bax.Limb('right')
        num_steps = left_traj.shape[0]
        assert left_traj.shape == right_traj.shape


        if move_to_start:
            print('Moving to start...')
            self.move_to_start(left_traj, right_traj)

        if prompt:
            raw_input('Ready. Press enter to begin. ')
        print('Moving...')
        # prepare recorder:
        self.recorder.start(hz, secs)
        for t in range(self.recorder.num_frames):
            lpose, rpose = arr_to_pose(left_traj[t,:], 'left'), arr_to_pose(right_traj[t,:], 'right')
            left.set_joint_positions(lpose)
            right.set_joint_positions(rpose)
            
            self.recorder.record_frame(t)

            rate.sleep()
        # stop checking for collisions to avoid overrunning:
        self.recorder.audiorec.enabled = False

        toc = time.time()
        rostoc = rospy.get_time()
        self.recorder.save(filename)

        print ('Done moving. Processing data...')

        time_taken = toc-tic
        rostime_taken = rostoc - rostic

        print("""Trial %s recorded. 
ROS time taken: %.2f (desired: %.2f)
Real time taken: %.2f (real time factor: %.2f)""" % (filename, rostime_taken, (secs - (1.0/hz)), time_taken, rostime_taken / time_taken))

def beat_dict_duration(beat_dict):
    """get the time in seconds required to play a beat dict"""
    times = [k[-1] for k in beat_dict.values()]
    max_time = np.max(times) + 0.2
    return max_time

def generate_tab(left_time, right_time, left_seq, right_seq, duration):
    beats = {}
    num_left_drums = len(left_seq)
    num_right_drums = len(right_seq)
    i = 0
    for drum_name in left_seq:
        beats[drum_name] = np.arange(i*left_time, duration, left_time*num_left_drums)
        i += 1
    i = 0
    for drum_name in right_seq:
        beats[drum_name] = np.arange(i*right_time, duration, right_time*num_right_drums)
        i += 1
    return beats

if __name__ == '__main__':
    rospy.init_node('record_playback')
    bax.RobotEnable(True).enable()
    rospy.loginfo('Ready to go.')

    filename = 'example2.mmd'

    # parameters for generating example tabs:

    duration = 3.0

    time_pairs = [[0.4, 0.4], [0.5, 0.5], [0.6, 0.6], [0.7, 0.7], [0.8, 0.8], [0.9, 0.9],
                  [0.4, 0.8], [0.8, 0.4], [0.4, 0.6], [0.6, 0.4], [0.6, 0.8], [0.8, 0.6]]

    left_seqs = [['cymbal1'], ['tom1']]    # movement to and from other drums is tricky
    right_seqs = [['snare', 'cymbal2'], ['snare'], ['cymbal2']]


    pr = PlaybackRecorder(img_dims=IMG_DIM)


    filenum = 1
    filepath = datadir + ('trial%03d.mmd' % filenum)

    num_trials = len(time_pairs)**2 * len(left_seqs) * len(right_seqs)

    i = 1
    for lt, rt in time_pairs:
        for lseq in left_seqs:
            for rseq in right_seqs:
                tab = generate_tab(lt, rt, lseq, rseq, duration)
                while os.path.isfile(filepath): # add trial file but do not overwrite existing files
                    filenum += 1
                    filepath = datadir + ('trial%03d.mmd' % filenum)

                print ('Recording trial %d of %d' % (i, num_trials))
                print('Left time: %s\nRight time: %s\nLeft seq: %s\nRight seq: %s' % (lt, rt, lseq, rseq))
                pr.play_and_record(tab, filepath, move_to_start=True, prompt=False, hz=HZ, secs=duration)
                i += 1


    # pr.play_and_record(hard_beats, datadir+filename, move_to_start=True, hz=HZ, secs=beat_dict_duration(hard_beats))

    mmd = load_mmd_obj(filename)