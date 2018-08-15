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
    def __init__(self, img_dims = [50,50,3], strike_time = 0.2, prompt=True):

        self.recorder = Recorder(camera_topic='/cameras/head_camera/image', 
                    collision_topic='drum_collision',
                    img_dim=[800,800,3], 
                    downsampled_img_dim=img_dims)
        self.strike_time = strike_time
        self.beater = DrumBeater(drum_arms, drum_patterns, self.strike_time)
        self.prompt = prompt

    def move_to_start(self, left_traj, right_traj):
        """move to the starting position of a given trajectory"""
        move_to_pose(arr_to_pose(left_traj[0,:], 'left'), arr_to_pose(right_traj[0,:], 'right'))        

    def play_and_record(self, beats, filename, secs, hz=HZ, move_to_start=True):
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

        if self.prompt:
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

if __name__ == '__main__':
    rospy.init_node('record_playback')
    bax.RobotEnable(True).enable()
    rospy.loginfo('Ready to go.')

    filename = 'example2.mmd'

    # some beat dicts for testing:
    example_beats = {'snare': np.arange(0,10, 2.), 
                     'cymbal2': np.arange(1,11, 2.),
                     'hihat': np.arange(0,11,1.)}

    hard_beats = {'snare': np.arange(0.8, 2, 1.2),
                  'cymbal2': np.arange(0.2, 2, 1.2),
                  'hihat': np.arange(0.2, 2, 0.6)}

    easy_beats = {'snare': np.arange(1, 8, 1.2),
                  'cymbal2': np.arange(1.6, 8.6, 1.2)}

    quick_beats = {'snare': np.arange(0.4, 2.0, 0.4),
                    'hihat': np.arange(0.4, 2.0, 0.6)}

    pr = PlaybackRecorder(img_dims=IMG_DIM)
    pr.play_and_record(hard_beats, datadir+filename, move_to_start=True, hz=HZ, secs=beat_dict_duration(hard_beats))

    mmd = load_mmd_obj(datadir+filename)