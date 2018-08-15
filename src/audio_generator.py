import rospy 
from baxter_core_msgs.msg import EndpointState
import baxter_interface as bax
import numpy as np
import time
import pygame
import sound_utils as snd # my module for making waveforms of audio data
from drum_surfaces import drum_soundfiles

# monitor left end effector for drum contact:
ltopic = "robot/limb/left/endpoint_state"
rtopic = "robot/limb/right/endpoint_state"

SOUND_RATE = 44100 # sound sampling rate in hz

def synth_sound(duration, collision_dict):
    """given a recording of some duration and the resultant move/collision indices, 
    synthesise the waveform. we expect collisions and move_onsets etc to be lists
    of indices in the 44100 hz waveform of the given duration"""

    # use gaussian white noise as the base:
    wave = snd.white_noise( duration=duration, 
                                    amplitude=500, 
                                    kind='gaussian')
    drum_names = collision_dict.keys()
    drum_wavs = {name:snd.load_wav(drum_soundfiles[name]) for name in drum_names}

    for name, col_list in collision_dict.iteritems():
        sound = drum_wavs[name]
        for beat_time in col_list:
            beat_frame = int(np.round(beat_time * SOUND_RATE))
            wave = snd.insert_sound(wave, sound, beat_frame)

    return wave

if __name__ == '__main__':
    # synthesise a test collision dict: 

    hard_beats = {'snare': np.arange(0.8, 2, 1.2),
              'cymbal2': np.arange(0.2, 2, 1.2),
              'hihat': np.arange(0.2, 2, 0.6)}

    sound = synth_sound(2, hard_beats)