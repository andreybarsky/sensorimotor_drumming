import rospy
import pickle
import numpy as np
import cv2
import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_mmd import MultimodalData
from audio_generator import synth_sound

import sys, os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time
from drumming.msg import collision

from drum_surfaces import drum_names 

class ImageRecorder(object):
    """listens to ros image data on camera_topic, captures downsampled images when called"""
    def __init__(self, camera_topic, input_dim, downsampled_dim):
        self.camera_topic = camera_topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.im_callback)

        self.input_dim = input_dim
        self.downsampled_dim = downsampled_dim
        self.current_frame = np.zeros(downsampled_dim)

        self.capturing = False
        rospy.loginfo('ImageRecorder initialised.')

    def im_callback(self, data):
        if self.capturing: # no need to do this when not recording
            # stores the current frame in a one-image buffer
            self.current_frame = data

    def capture(self):
        """Captures and returns one frame of image data"""
        cv_image = self.bridge.imgmsg_to_cv2(self.current_frame) #, desired_encoding="passthrough")
        resized_image = cv2.resize(cv_image, tuple(self.downsampled_dim[:2])) # cv2.resize takes only the height/width dimensions, not the num. of channels
        np_image = np.asarray(resized_image)
        return np_image

    def enable(self, switch):
        """enables or disables image capture
        if enabling, we also wait until we've received at least one image for the buffer"""
        self.capturing = switch
        if self.capturing:
            # make sure we have at least one frame before continuing
            self.current_frame = rospy.wait_for_message(self.camera_topic, Image, 2)

    def start(self): # wrapper for convenience
        self.enable(True)

    def stop(self):
        self.enable(False)


class JointRecorder(object):
    """listens to joint angles and captures their current state when called.
    not the same JointRecorder as in baxter_examples/recorder.py, but derivative of it"""
    def __init__(self):
        self.leftarm = baxter_interface.Limb("left")
        self.rightarm = baxter_interface.Limb("right")

        self.left_jointnames = self.leftarm.joint_names()
        self.right_jointnames = self.rightarm.joint_names()

        self.dof = (len(self.left_jointnames) + len(self.right_jointnames))

        rospy.loginfo('JointRecorder initialised.')

    def capture(self):
        jpos = np.zeros(self.dof) # joint positions
        jvel = np.zeros(self.dof) # joint velocities

        i = 0
        for j in self.left_jointnames: # make sure we capture them in correct order of jointnames
            jpos[i] = self.leftarm.joint_angles()[j]
            jvel[i] = self.leftarm.joint_velocities()[j]
            i += 1
        for j in self.right_jointnames:
            jpos[i] = self.rightarm.joint_angles()[j]
            jvel[i] = self.rightarm.joint_velocities()[j]
            i += 1

        # # end effector poses:
        # lxe_pose = self.leftarm.endpoint_pose()
        # rxe_pose = self.rightarm.endpoint_pose()
        # xepos = list(lxe_pose['position']) + list(lxe_pose['orientation']) + \
        #         list(rxe_pose['position']) + list(rxe_pose['orientation'])

        # # end effector velocities:
        # lxe_vel = self.leftarm.endpoint_velocity()
        # rxe_vel = self.rightarm.endpoint_velocity()
        # xevel = list(lxe_vel['linear']) + list(lxe_vel['angular']) + \
        #         list(rxe_vel['linear']) + list(rxe_vel['angular'])

        joints = np.concatenate((jpos, jvel)) # , np.array(xepos), np.array(xevel)))
        return joints
            

class AudioRecorder(object):
    """listens to audio and records a np matrix of raw sound waveforms
    implementation is very different between simulation and RL:
        - in simulation we subscribe to joint angles to detect collisions between robot and drums,
        and synthesise some fake data to represent the resulting sounds.
        - in RL we'll just listen to the microphone we have set up (not done yet)"""
    def __init__(self, collision_topic=None, hz=44100, simulation=True):
        self.simulation = simulation
        self.collision_topic = collision_topic # topic for simulated drum collisions if applicable
        self.enabled = False
        self.colsub = rospy.Subscriber(collision_topic, collision, self.collision_callback)

    def start(self):
        """listen to drum collision topic for x seconds and record when they happen"""
        assert self.collision_topic is not None
        self.enabled=True
        self.start_time = rospy.get_time()
        self.collision_times = {} # dict of drum collision times, keyed by drum name
        for name in drum_names: # fill in the dict
            self.collision_times[name] = []

    def stop(self):
        self.enabled=False

    def collision_callback(self, data):
        if self.enabled:
            # record drum name and time of impact:
            self.collision_times[data.drum].append(rospy.get_time() - self.start_time)

    def synth_audio(self, secs):
        """take list self.collision_times and output an audio waveform"""
        self.enabled = False

        # clean up empty lists if no beat on that drum was detected:
        for d in drum_names:
            if len(self.collision_times[d]) == 0:
                del self.collision_times[d]

        return synth_sound(secs, self.collision_times)


class Recorder(object):
    """Main recorder class that collates data from all modalities"""
    def __init__(self, camera_topic, collision_topic, img_dim, downsampled_img_dim):


        # subrecorders:
        self.imgrec = ImageRecorder(camera_topic, img_dim, downsampled_img_dim)
        self.audiorec = AudioRecorder(collision_topic)
        self.jointrec = JointRecorder()

        self.imgdim = downsampled_img_dim # dimensions of the downsampled image

        self.jointdim = 2 * self.jointrec.dof # degrees of freedom of arm movements, including velocity

        rospy.loginfo('Main recorder initialised.')

    def record_frame(self, framenum):      
        """Captures a single frame of data from each subrecorder and stores in class.
        Relies on self.imgrec and self.jointrec existing and being initialised."""  
        timestamp = rospy.get_time()

        img = self.imgrec.capture()
        joints = self.jointrec.capture()
        # print('joints shape: %s' % str(joints.shape))
        
        # audio rec doesn't return anything, just listens to the collision topic

        # add those values to the full array with timestamp:
        self.imgbuffer[framenum] = img
        # rospy.loginfo('Image captured at time: %.5f' % rospy.get_time())
        # print('Jointbuffer shape: %s' % str(self.jointbuffer.shape))
        self.jointbuffer[framenum] = joints
        # rospy.loginfo('Joint angles captured at time: %.5f' % rospy.get_time())
        self.timestamps[framenum] = timestamp

    def start(self, hz, secs):
        self.hz = hz
        self.secs = secs
        self.num_frames = int(np.floor(hz*secs))

        self.rate = rospy.Rate(self.hz)

        self.clear_buffer() # initialise image and joint buffers

        self.imgrec.start()
        self.audiorec.start()
        self.jointrec.__init__()
        # joint recorder doesn't need starting

    def save(self, filename):
        """end recording and save data to .mmd file"""
        self.imgrec.stop()
        self.audiorec.stop()

        self.audio = self.audiorec.synth_audio(self.secs)
        # self.audio = np.zeros((self.secs * 44100))

        joint_names = self.jointrec.leftarm.joint_names()
        joint_names += self.jointrec.rightarm.joint_names()
        joint_names += [j+ 'vel' for j in self.jointrec.leftarm.joint_names()] 
        joint_names += [j+ 'vel' for j in self.jointrec.rightarm.joint_names()] 

        # package them together for saving:
        header = {'name': filename,
            'framerate': self.hz, 
            'frames' : self.num_frames, 
            'img_dims': self.imgdim, 
            'joint_names': joint_names}

        data = MultimodalData(image=self.imgbuffer, joints=self.jointbuffer, audio=self.audio, 
            times=self.timestamps, header=header, collisions = self.audiorec.collision_times)

        rospy.loginfo('Multimodal data collated.')

        with open(filename, 'wb') as outfile:
            pickle.dump(data, outfile, -1)
            rospy.loginfo('Multimodal data object saved to %s' % filename)
        self.clear_buffer()

    def record_trial(self, secs, hz, filename):
        """Records a trial of multimodal data at the resolution specified by hz and secs, and saves to filename"""
        # enable image recorder:
        self.start(secs, hz)

        # recording loop:
        for f in range(self.num_frames):
            self.record_frame(f)
            self.rate.sleep()
        
        self.save(filename)

    def clear_buffer(self):
        """Clears data buffer"""
        self.imgbuffer = np.zeros([self.num_frames] + self.imgdim)  # (hz*secs) x 800 x 800 x 3 array
        self.jointbuffer = np.zeros((self.num_frames, self.jointdim))
        self.timestamps = np.zeros(self.num_frames)


# example usage:
def main():
    main_rec = Recorder(camera_topic='/cameras/head_camera/image', 
                        collision_topic='drum_collision',
                        img_dim=[800,800,3], 
                        downsampled_img_dim=[50,50,3])

    # ready to record:
    rospy.sleep(0.5) # give it a moment to latch subscribers etc.

    datadir = os.getcwd() + '/trials/'

    print('Ready to record first trial. Press enter to proceed.')
    sys.stdin.readline()
    # trial recording loop:
    try:
        while True:
            filenum = 1
            filename = datadir + ('trial%03d.mmd' % filenum)

            while os.path.isfile(filename): # add trial file but do not overwrite existing files
                filenum += 1
                filename = datadir + ('trial%03d.mmd' % filenum)
            
            rostic = rospy.get_time()
            tic = time.time()

            main_rec.record_trial(filename=filename, hz=20, secs=2) # main loop
            
            toc = time.time()
            rostoc = rospy.get_time()

            time_taken = toc - tic
            rostime_taken = rostoc - rostic
            print("""Trial %03d recorded. 
    ROS time taken: %.2f (desired: %.2f)
    Real time taken: %.2f (real time factor: %.2f)
        Press enter for new trial, or Ctrl+C, enter to exit.""" % (filenum, rostime_taken, (main_rec.secs - (1.0/main_rec.hz)), time_taken, rostime_taken / time_taken))
            sys.stdin.readline()
    except KeyboardInterrupt:
        print('Closing trial recorder.')
        
if __name__ == '__main__':
    rospy.init_node('data_collector')
    main()