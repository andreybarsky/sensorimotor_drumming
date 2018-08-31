# classes and functions for using and loading multimodal robot data

import numpy as np
import pickle
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.mlab import PCA
from matplotlib import gridspec
from copy import deepcopy
import random

class MultimodalData(object):
    """Holds image and joint data together in one object with methods for easy visualisation"""
    def __init__(self, image, joints, audio, collisions, times, header):
        self.header = header
        self.image = image
        self.joints = joints
        self.collisions = collisions
        self.times = times

        if len(audio.shape) == 1: # if audio is a vector we reshape it:
            self.audio = self.bin_audio(audio) # into matrix where each row corresponds to a video frame / time slice
        else:
            self.audio = audio

    def pca_joints(self, num, data=None):
        """compresses joint data using PCA and returns a projection onto num axes.
        optionally, PCA weights can be computed based on another dataset.
        if "collisions", also highlight the points where drum collisions are detected"""
        if data is None:
            data = self.joints
        lpca = PCA(data[:,:7]) # joint positions only
        rpca = PCA(data[:,7:14])

        # extract 'num' components
        lproj = lpca.project(self.joints[:,:7], minfrac=lpca.fracs[num-1])
        rproj = rpca.project(self.joints[:,7:14], minfrac=rpca.fracs[num-1])

        return np.concatenate((lproj, rproj), axis=1)

    def show_joints(self, collisions=True, vertical=False, data=None):
        """plot joint angles over time using PCA"""
        hz = self.header['framerate']

        # plot PCA-reduced joints, either sideways or not::
        frames = np.arange(self.header['frames'])
        pca_joint_data = self.pca_joints(1, data=data)
        if not vertical:
            plt.plot(frames, pca_joint_data)
        else:
            plt.plot(pca_joint_data, frames)

        if collisions:
            left_beat_frames = []
            right_beat_frames = []
            drum_arms ={'hihat'  : 'left' ,
                        'cymbal1': 'left' ,
                        'tom1'   : 'left' ,
                        'snare'  : 'right',
                        'cymbal2': 'right',
                        'tom2'   : 'right'}

            # get collision times, convert them into frame indices for each arm:
            for drum_name, beat_times in self.collisions.items():
                for t in beat_times:
                    beat_frame = int(np.round(t*hz))
                    if drum_arms[drum_name]=='left':
                        left_beat_frames.append(beat_frame)
                    else:
                        right_beat_frames.append(beat_frame)
            plt.plot(left_beat_frames, pca_joint_data[left_beat_frames,0], 'o')
            plt.plot(right_beat_frames, pca_joint_data[right_beat_frames,1], 'o')
        plt.show()



    def bin_audio(self, audio):
        """Slices a 44100 hz audio wave into chunks.
        The result is a n * m matrix where n is the number of frames and m is the length of each chunk"""
        frames = self.header['frames']
        sound_samples_per_frame = len(audio) / frames

        print('==\n number of frames: %d\n number of sound samples: %d \n==' % (frames, sound_samples_per_frame))

        return np.reshape(audio, (frames, sound_samples_per_frame))

    def init_video(self, ax, dims=(50,50,3), texts=True, axlabels=True):
        vaxplot = ax.imshow(np.zeros(dims), animated=True)
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)

        if texts:
            #timetext = ax.text(1, 2, '', animated=True, fontsize=10)
            #frametext = ax.text(45, 2, '', animated=True, fontsize=10)
            titletext = ax.text(3, 5, '', animated=True, fontsize=10)
            return vaxplot, titletext # , timetext, frametext
        else:
            return vaxplot

    def init_audio(self, ax, spectro=False, axlabels=True):
        saxplot, = ax.plot([], color='#000000', linewidth=0.5, animated=True)
        if not spectro:
            fps = self.header['framerate']
            sound_samples_per_frame = int((1.0 / fps) / (1.0 / 44100))
            max_amp = np.max(self.audio) + 2000
            ax.set_xlim([0, sound_samples_per_frame])
            ax.set_ylim([-max_amp, max_amp])
            ax.xaxis.set_visible(False)
            if axlabels:
                ax.set_ylabel('Magnitude')
            else:
                ax.xaxis.set_visible(False)
                ax.yaxis.set_visible(False)
        elif spectro:
            full_audio = np.reshape(self.audio, np.product(self.audio.shape))
            full_fourier = np.fft.fft(full_audio)
            nf = len(full_audio)
            full_fourier /= float(nf)

            # khz array is used inside update_frame as the x axis:
            n = len(self.audio[0])
            freq_array = np.arange(0, n/2, 1.0) * (44100 * 1.0 / n)
            self.khz_array = freq_array / 1000

            max_db = np.max(np.real(10*np.log10(full_fourier)))
            min_db = np.min(np.real(10*np.log10(full_fourier)))
            ax.set_xlim([np.min(self.khz_array), np.max(self.khz_array)])
            ax.set_ylim([min_db-5, max_db+5])
            if axlabels:
                ax.set_xlabel('kHz')
                ax.set_ylabel('dB')
            else:
                ax.xaxis.set_visible(False)
                ax.yaxis.set_visible(False)

        return saxplot

    def init_joints(self, ax, axlabels=True):
        jaxplotl, = ax.plot(np.zeros(7), np.arange(0,7), color='blue', marker='o', animated=True)
        jaxplotr, = ax.plot(np.zeros(7), np.arange(0,7), color='red', marker='o', animated=True)

        left_pos = 4
        right_pos = 8
        xsize = 12

        ax.set_xlim([0,xsize])
        ax.set_xticks([left_pos,right_pos])
        ax.set_ylim([-0.2, 6.2])

        # general joint names: (s0, s1, e0 etc.)
        ambi_jnames = [x[-2:] for x in self.header['joint_names'][:7]]
        ax.set_yticklabels([''] + ambi_jnames)
        arm_names = ['left', 'right']
        #arm_names = [''] * xsize
        #arm_names[2] = 'left'
        #arm_names[4] = 'right'
        if axlabels:
            ax.set_xticklabels(arm_names)
        else:
            ax.xaxis.set_visible(False)
            ax.yaxis.set_visible(False)

        ax.axvline(left_pos, linestyle='--', color='blue')
        ax.axvline(right_pos, linestyle='--', color='red')

        return jaxplotl, jaxplotr

    def update_frame(self, f, spectro=True):
        # video:
        img = self.image[f]
        self.vaxplot.set_data(img / 255)
        # self.timetext.set_text('%.1f' % self.times[f])
        # self.frametext.set_text('%02d' % f)

        # audio:
        if not spectro: # plot raw audio
            aud_frame = self.audio[f]
            self.saxplot.set_data(np.arange(0, len(aud_frame)), aud_frame)
        else: # plot spectrogram
            aud_frame = self.audio[f]
            n = len(aud_frame)
            fourier = np.fft.fft(aud_frame)[0:int(np.ceil(n/2))] / float(n)
            self.saxplot.set_data(self.khz_array, np.real(10*np.log10(fourier)))

        # joints: (we need to create the vectors that let us plot joint angles as lines)
        left_pos = 4
        right_pos = 8
        jleft = self.joints[f,:7]
        jright = self.joints[f,7:14]
        xleft = (np.ones(7)*left_pos) + jleft
        xright = (np.ones(7)*right_pos) + jright
        vert = np.arange(0,7)
        self.jaxplotl.set_data(xleft, vert)
        self.jaxplotr.set_data(xright, vert)

        plots = (self.vaxplot, self.saxplot, self.jaxplotl, self.jaxplotr)

        return plots

    def show(self, dims=(50,50,3), spectro=True, framerate = None):
        """Animates this mmd object with video, audio and joint angles.
        Animation update rate is framerate, if None it defaults to the fps that was used
        for the recording."""
        fig = plt.figure()
        # initialise axes:
        self.vax = plt.subplot2grid((4,4), (0,0), colspan=3, rowspan=3)
        self.sax = plt.subplot2grid((4,4), (3,0), colspan=3, rowspan=1)
        self.jax = plt.subplot2grid((4,4), (0,3), colspan=1, rowspan=4)

        # initialise plots:
        self.vaxplot = self.init_video(self.vax, dims=dims, texts=False)
        self.saxplot = self.init_audio(self.sax, spectro=spectro)
        self.jaxplotl, self.jaxplotr = self.init_joints(self.jax)

        if framerate is None:
            framerate = self.header['framerate']
        interval = 1000 / framerate
        frames = self.header['frames']

        ani = animation.FuncAnimation(fig, self.update_frame, frames=frames,
                                      interval=interval, blit=True, fargs=[spectro])
        plt.show()

    def compare(self, mmd_list, cols=None, spectro=False, framerate=None, titles = None, axlabels=False):
        """Compare multimodal visualisations of this object and other object passed as function args"""
        self.datas = [self] + mmd_list
        self.titles = titles
        num_datas = len(self.datas)

        # determine size of grid:
        if cols is None:
            cols = min(num_datas, 3) # have 3 visualisations per row as default
        rows = int(np.ceil(num_datas / cols))
        cell_size = (4,4) # size of each mmd visualisation 'cell'
        grid_size = (rows*cell_size[0], cols*cell_size[1])

        # determine corners of each cell:
        corner_xs = np.arange(0, grid_size[0], cell_size[0])
        corner_ys = np.arange(0, grid_size[1], cell_size[1])
        # cartesian product:
        corners = [(x, y) for x in corner_xs for y in corner_ys][:num_datas]

        # initialise the figure:
        fig = plt.figure()
        self.vaxes = [] # video axes
        self.saxes = [] # sound axes
        self.jaxes = [] # joint axes
        self.vaxplots = [] # contains data for each video axis
        self.saxplots = []
        self.jaxplotsl = []
        self.jaxplotsr = []
        self.titletexts = []

        # loop and append each mmd cell's various axes to the axes lists:
        for i in range(num_datas):
            vax = plt.subplot2grid(grid_size, corners[i], colspan=3, rowspan=3)
            self.vaxes.append(vax)

            vaxplot, titletext = self.init_video(vax, texts=True, axlabels=axlabels)
            self.vaxplots.append(vaxplot)
            self.titletexts.append(titletext)
            #self.timetexts.append(timetext)
            #self.frametexts.append(frametext)

            sax = plt.subplot2grid(grid_size, (corners[i][0]+3, corners[i][1]), colspan=3, rowspan=1)
            self.saxes.append(sax)
            saxplot = self.init_audio(sax, spectro=spectro, axlabels=axlabels)
            self.saxplots.append(saxplot)

            jax = plt.subplot2grid(grid_size, (corners[i][0], corners[i][1]+3), colspan=1, rowspan=4)
            self.jaxes.append(jax)
            jaxplotl, jaxplotr = self.init_joints(jax, axlabels=axlabels)
            self.jaxplotsl.append(jaxplotl)
            self.jaxplotsr.append(jaxplotr)

        if framerate is None:
            framerate = self.header['framerate']
        interval = 1000 / framerate
        frames = self.header['frames']

        ani = animation.FuncAnimation(fig, self.update_multiframe, frames=frames,
                                      interval=interval, blit=True, fargs=[spectro])
        plt.show()


    def update_multiframe(self, f, spectro=True):
        # video:
        imgs = [m.image[f] for m in self.datas]
        auds = [m.audio[f] for m in self.datas]
        jlefts = [m.joints[f,:7] for m in self.datas]
        jrights = [m.joints[f,7:14] for m in self.datas]

        for i in range(len(self.datas)):
            self.vaxplots[i].set_data(imgs[i] / 255)
            # self.timetexts[i].set_text('%.1f' % self.times[f])
            # self.frametexts[i].set_text('%02d' % f)

            self.titletexts[i].set_text(self.titles[i])

            # audio:
            if not spectro: # plot raw audio
                aud_frame = auds[i]
                self.saxplots[i].set_data(np.arange(0, len(aud_frame)), aud_frame)
            else: # plot spectrogram
                aud_frame = auds[i]
                n = len(aud_frame)
                fourier = np.fft.fft(aud_frame)[0:int(np.ceil(n/2))] / float(n)
                self.saxplots[i].set_data(self.khz_array, np.real(10*np.log10(fourier)))

            # joints: (we need to create the vectors that let us plot joint angles as lines)
            left_pos = 4
            right_pos = 8
            jleft = jlefts[i]
            jright = jrights[i]
            xleft = (np.ones(7)*left_pos) + jleft
            xright = (np.ones(7)*right_pos) + jright
            vert = np.arange(0,7)
            self.jaxplotsl[i].set_data(xleft, vert)
            self.jaxplotsr[i].set_data(xright, vert)

        plots = self.vaxplots + self.saxplots + self.jaxplotsl + self.jaxplotsr + self.titletexts #  + self.timetexts + self.frametexts
        return plots


    def play_audio(self):
        pass # not implemented

    def compare_video(self, array_list, titles=None, framerate=None, num_cols=2):
        """given own data and a set of arrays of similarly-sized image data,
        display the two side-by-side animated across frame dimension"""
        datas = [self.image] + array_list
        assert datas[0].shape == datas[1].shape # we have at least two arrays and they are the same size

        fig = plt.figure()
        num_arrays = len(datas)
        num_rows = int(np.ceil(len(datas) / num_cols))

        self.axes = []
        self.axplots = []
        for x in range(len(datas)):
            row_num = int(np.floor(x / num_cols))
            col_num = x % num_cols
            self.axes.append(plt.subplot2grid((num_rows,num_cols), (row_num,col_num), colspan=1, rowspan=1))
            self.axplots.append(self.init_video(self.axes[-1], texts=False))
            if titles is not None:
                self.axes[x].set_title(titles[x])

        if framerate is None:
            framerate = self.header['framerate']
        interval = 1000 / framerate
        frames = self.header['frames']

        def update(f):
            for x in range(len(datas)):
                img = datas[x][f]
                self.axplots[x].set_data(img / 255)
            return self.axplots

        ani = animation.FuncAnimation(fig, update, frames=frames,
                                      interval=interval, blit=True)
        plt.show()


    def save(self, filename):
        with open(filename, 'wb') as outfile:
            pickle.dump(self, outfile, 2)

class DataBatcher(object):
    """Batch generator for network training on image or audio data"""
    # we expect data in the shape [trials, frames, x, y, channel] for video
    # or [trials, frames, magnitude] for audio (either raw or spectrograms)
    def __init__(self, data, test_perc = 0.2, seed = None):
        """args
        data:: the full ndarray of whatever data we're working with
        test_perc:: the proportion of data that we reserve for the test set
        seed:: a random seed to use, for reproducible shuffling / test data slicing"""

        # first allocate all data and randomly shuffle it:
        self.data = data
        np.random.seed(seed)
        self.shuffle() 

        num_total_trials = data.shape[0]
        self.num_test_trials = int(num_total_trials * test_perc)
        
        # then split off training and test sets:
        # and reallocate self.data to mean training data specifically:
        self.test_data = self.data[:self.num_test_trials]
        self.data = self.data[self.num_test_trials:]

        self.shape = self.data.shape
        self.num_trials = self.shape[0]

        self.current_idx = 0

        self.shuffle()

    def shuffle(self):
        """Shuffles across first axis"""
        num_trials = self.data.shape[0]
        perm = np.arange(num_trials)
        np.random.shuffle(perm)

        self.data = self.data[perm] # reallocate data to indices perm

    def standardise(self):
        pass

    def unstandardise(self):
        pass

    def next_batch(self, batch_size):
        """Returns next batch of data trials."""
        old_idx = self.current_idx
        self.current_idx += batch_size

        if self.current_idx > self.num_trials: # we need to get part of the data, shuffle and get the rest
            first_data = self.data[old_idx:]
            self.shuffle()

            rest_idx = self.current_idx - self.num_trials
            self.current_idx = rest_idx

            rest_data = self.data[:rest_idx]

            batch_data = np.concatenate((first_data, rest_data))

        else:
            batch_data = self.data[old_idx:self.current_idx]

        # make sure this batch is the size we asked for:
        assert batch_data.shape[0] == batch_size
        return batch_data

    def save(self, filename):
        with open(filename, 'wb') as outfile:
            pickle.dump(self, outfile, 2)


class MmdBatcher(object):
    """generalisation of databatcher class for data with multiple modalities that needs to be kept together"""
    def __init__(self, data, data_type='arrays', standardise_inputs=False, test_perc = 0.2, seed=None):
        """args
        data:: the full data of whatever data we're working with, as specified by data_type
        data_type:: if 'mmd', then the data is an mmd object with image, audio and joints attributes
                    if 'arrays', then the data is a tuple of ndarrays (image, audio, joints)
        test_perc:: the proportion of data that we reserve for the test set"""

        # first allocate all data and randomly shuffle it:
        
        if data_type == 'arrays':
            self.image, self.audio, self.joints = data
        elif data_type == 'mmd': 
            self.image = data.image
            self.audio = data.audio
            self.joints = data.joints
        self.datas = (self.image, self.audio, self.joints)

        self.standardise_inputs = standardise_inputs
        self.means = [np.mean(x) for x in self.datas]
        self.stdevs = [np.std(x) for x in self.datas]
        if self.standardise_inputs: 
            self.datas = self.standardise(self.datas)

        np.random.seed(seed)
        self.shuffle() 

        num_total_trials = self.image.shape[0]
        assert num_total_trials == self.audio.shape[0] and num_total_trials == self.joints.shape[0]

        num_test_trials = int(num_total_trials * test_perc)
        self.num_test_trials = num_test_trials
        
        # then split off training and test sets:
        self.test_image = self.image[:num_test_trials]
        self.image = self.image[num_test_trials:]

        self.test_audio = self.audio[:num_test_trials]
        self.audio = self.audio[num_test_trials:]

        self.test_joints = self.joints[:num_test_trials]
        self.joints = self.joints[num_test_trials:]

        self.datas = (self.image, self.audio, self.joints)
        self.test_datas = (self.test_image, self.test_audio, self.test_joints)

        self.num_trials = self.image.shape[0]

        self.current_idx = 0

        self.shuffle()

    def shuffle(self):
        """shuffle all data structures along first axis with same permutation for each"""
        num_trials = self.image.shape[0]
        perm = np.arange(num_trials)
        np.random.shuffle(perm)

        self.image = self.image[perm]
        self.audio = self.audio[perm]
        self.joints = self.joints[perm]

    def next_batch(self, batch_size):
        """Returns next batch of data trials."""
        old_idx = self.current_idx
        self.current_idx += batch_size

        if self.current_idx > self.num_trials: # we need to get part of the data, shuffle and get the rest
            first_img = self.image[old_idx:]
            first_aud = self.audio[old_idx:]
            first_jnt = self.joints[old_idx:]
            self.shuffle()

            rest_idx = self.current_idx - self.num_trials
            self.current_idx = rest_idx

            rest_img = self.image[:rest_idx]
            rest_aud = self.audio[:rest_idx]
            rest_jnt = self.joints[:rest_idx]
            
            batch_img = np.concatenate((first_img, rest_img))
            batch_aud = np.concatenate((first_aud, rest_aud))
            batch_jnt = np.concatenate((first_jnt, rest_jnt))

        else:
            batch_img = self.image[old_idx:self.current_idx]
            batch_aud = self.audio[old_idx:self.current_idx]
            batch_jnt = self.joints[old_idx:self.current_idx]

        return (batch_img, batch_aud, batch_jnt)

    def standardise(self, data, mode='all'):
        """standardise some data according to the stored parameters of this databatcher object
        if mode='all', we expect datas to be a tuple of (video, audio, joints)
        or mode can be 'image', 'audio' or 'joints' to standardise just one array"""

        if mode == 'all':
            datas = []
            for i in range(3):
                std_data = (data[i] - self.means[i]) / self.stdevs[i]
                datas.append(std_data)
            return tuple(datas)
        elif mode in ['image', 'video']:
            return (data - self.means[0]) / self.stdevs[0]
        elif mode in ['audio', 'sound']:
            return (data - self.means[1]) / self.stdevs[1]
        elif mode in ['joints', 'joint']:
            return (data - self.means[2]) / self.stdevs[2]
        else:
            raise Exception('invalid mode argument to standardise')

    def unstandardise(self, data, mode='all'):

        if mode=='all':
            for i in range(3):
                data = (data[i] * self.stdevs[i]) + self.means[i]
                return data
        elif mode in ['image', 'video']:
            return (data * self.stdevs[0]) + self.means[0]
        elif mode in ['audio', 'sound']:
            return (data * self.stdevs[1]) + self.means[1]
        elif mode in ['joints', 'joint']:
            return (data * self.stdevs[2]) + self.means[2]
        else:
            raise Exception('invalid mode argument to standardise')



def load_dir_as_arrays(datadir, numtrials):
    #loads a series of multimodal datafiles into numpy matrices

    # append final slash to data directory if needed:
    if datadir[-1] != '/':
        datadir = datadir + '/'

    fnames = ['trial%03d.mmd' % i for i in range(1, numtrials+1)]
    f = [datadir + name for name in fnames]

    datas = []
    for filename in f:
        try:
            with open(filename, 'rb') as file:
                # python 2 and 3 unpickle differently:
                if sys.version_info > (3,0): # detect python3
                    datas.append(pickle.load(file, encoding='latin1'))
                else:
                    datas.append(pickle.load(file))
        except:
            print('Error opening file %s, ignoring.' % filename)

    numtrials = len(datas)

    # np array of dims [trial, frame, x, y, channel]
    imagetrials = np.zeros([numtrials] + list(datas[0].image.shape))
    audiotrials = np.zeros([numtrials] + list(datas[0].audio.shape))
    jointtrials = np.zeros([numtrials] + list(datas[0].joints.shape))

    for x in range(numtrials):
        imagetrials[x,:] = datas[x].image
        audiotrials[x,:] = datas[x].audio
        jointtrials[x,:] = datas[x].joints

    return imagetrials, audiotrials, jointtrials

def load_mmd_obj(filename):
    """loads provided mmd file as MultimodalData object"""
    with open(filename, 'rb') as file:
        if sys.version_info > (3,0):
            data = pickle.load(file, encoding='latin1')
        else:
            data = pickle.load(file)
    return data


def drop_modes(data_tuple, num_to_drop=[0,1,2]):
    """Randomly zero out data across a tuple of arrays, such that one of num_to_drop
    in the tuple is missing for each element in the batch, independently across batches.
    i.e. if num_to_drop=[0,1,2], we zero either 0, 1 or 2 modalities at random."""

    drop_scenarios = [] # possible things we could do to each data point in the batch
    
    newdata = deepcopy(data_tuple) # ensure that we're not modifying the original data
                                   # (this has happened)

    if not isinstance(num_to_drop, (list,tuple)):
        num_to_drop = [num_to_drop] # wrap int in list for simplicity
    for i in num_to_drop:
        if i == 0:
            drop_scenarios.append((1,1,1)) # 1 for keep, 0 for drop
        if i == 1:
            drop_scenarios += [(1,1,0), (1,0,1), (0,1,1)]
        if i == 2:
            drop_scenarios += [(1,0,0), (0,1,0), (0,0,1)]
    
    batch_size = data_tuple[0].shape[0]
    for b in range(batch_size):
        which_drop = random.choice(drop_scenarios)
        
        for m in range(3):
            if which_drop[m] == 0:
                newdata[m][b] *= 0

    return newdata

if __name__=='__main__':
    mmd = load_mmd_obj('trials/trial001.mmd')
    mmd.show_joints()