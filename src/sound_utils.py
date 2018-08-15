
# various util functions for generating and displaying sound data

import numpy as np
import subprocess as sp
import time
import pickle
import random
import matplotlib.pyplot as plt
import wavio # for 24 bit wav

#import scipy.io.wavfile as wv
import wavfile_enhanced as wv
from scipy import signal

mp3file = "snare3.mp3"
wavfile = "snare.wav"

hz = 44100 # standard CD audio sampling rate
channels = 1

def ffmpeg_read(soundfile):
    """read directly from system audio output"""
    ffmpeg = 'ffmpeg' # on linux; .exe on windows

    command = [ ffmpeg,
    '-i', soundfile,
    '-f', 's16le',
    '-acodec', 'pcm_s16le', # signed 16 bit PCM
    '-ar', '44100',
    '-ac', '2', # 2 channels
    '-']

    pipe = sp.Popen(command, stdout=sp.PIPE, bufsize=10**8)
    raw_audio = pipe.stdout.read(hz*channels*4)

    audio_array = np.fromstring(raw_audio, dtype='int16')
    reshaped_audio = audio_array # .reshape((len(audio_array)/2,2))

    return reshaped_audio

def play_sound(snd_array):
    import pygame # this is here because pygame is buggy
    pygame.init()
    pygame.mixer.init(hz, -16, 2) # 44100 hz, 16 bit
    if 1 in snd_array.shape: # convert to 2 channels
        snd_array = np.array(zip(snd_array[0,::2], snd_array[0,1::2]))
    elif len(snd_array.shape) == 1:
        # pygame is very stupid and reads wave amplitudes row wise
        # instead of column wise, even for mono sound!
        # just trust me on this:
        snd_array = np.array(list(zip(snd_array[::2], snd_array[1::2])))
    sound = pygame.sndarray.make_sound(snd_array)
    sound.play()

def load_wav(wavfile, flatten=True):
    """reads a wav file from file and returns as ndarray.
    if flatten, average multi-channel sound into one channel"""
    # try:
    wav = wv.read(wavfile)[1]
    wav_arr = np.array(wav, dtype=np.int16)
    if flatten:
        wav_arr = np.mean(wav_arr, 1)
    return wav_arr
    # except ValueError:
    #     rate, sampwidth, wav_arr = wavio.readwav(wavfile)
    #     if flatten:
    #         wav_arr = np.mean(wav_arr, 1)
    #     return wav_arr

def white_noise(duration, amplitude=3000, kind='uniform', channels=1):
    """creates white noise of duration in secs across n channels"""
    samples = int(duration * hz)
    if kind in ['uniform', 'unif', 'u']:
        wn = np.random.randint(-amplitude, amplitude, [channels, samples])
    elif kind in ['gaussian', 'gauss', 'g', 'normal', 'norm', 'n']:
        wn = np.random.normal(0, amplitude/2, [channels, samples])
    if channels == 1:
        wn = wn[0]
    return wn.astype(np.int16)

def make_sines(duration, amplitude=20000, freqs = [262, 330, 392]):
    step = 1.0 / hz
    t = np.arange(0, duration, step)
    wave = np.zeros(len(t))
    for f in range(len(freqs)):
        omega = (2 * np.pi) * freqs[f]
        wave += np.array(amplitude * np.sin(omega*t))
    return wave.astype(np.int16)

def showfreq(x):
    """displays frequency space of a 1d signal"""
    f = np.fft.fft(x)[0:(n/2)] # first half is real part

def show(*x):
    """show/compare any number of 1d signals"""
    num = len(x)
    for i in range(num):
        signal = x[i] 
        plt.subplot(num, 1, i+1)
        plt.plot(signal)
    plt.show()

def insert_sound(base, insert, frame):
    """inserts a sound 'insert' into base sound 'base' at frame 'frame'"""
    base_len = len(base)
    insert_len = len(insert)
    endframe = frame + insert_len
    if endframe < base_len: # if the sound fits
        base[frame:endframe] += insert.astype(np.int16)
    else: # we have to slice what sound we can fit in
        partial_len = base_len - frame
        partial_insert = insert[:partial_len]
        base[frame:] += partial_insert.astype(np.int16)
    try:
        assert base_len == len(base) # check that we are dealing with an np vector!
        # if using regular python lists, this will just splice the insert into the middle
    except:
        raise Exception('inputs to insert_sound must be np arrays that can be added together')
    return base

def insert_looped_sound(base, loop, startframe, endframe):
    """given a sound sample 'loop', inserts a looped version of it between startframe
    and endframe into the base"""
    base_len = len(base)
    loop_len = len(loop)
    insert_len = endframe - startframe
    num_full_loops = insert_len // loop_len
    partial_len = insert_len % loop_len
    full_loops = np.tile(loop, num_full_loops)
    partial_loop = loop[:partial_len]

    insert = np.concatenate((full_loops, partial_loop)).astype(np.int16)
    base[startframe:endframe] += insert

    try:
        assert base_len == len(base) # check that we are dealing with an np vector!
        # if using regular python lists, this will have just spliced the insert into the middle
    except:
        raise Exception('inputs to insert_sound must be np arrays that support mathematical vector operations')
    return base

def sound_to_spec(sound, window_size=2205):
    """converts raw sound to spectrogram with non-overlapping windows"""
    n = len(sound)
    freq_array = np.arange(0, n/2, 1.0) * (44100 * 1.0 / n)
    khz_array = freq_array / 1000

    fourier = np.fft.fft(aud_frame)[0:(n/2)] / float(n)
    self.saxplot.set_data(self.khz_array, np.real(10*np.log10(fourier)))

def stft(sig, windows=40, return_all=True):
    """invertible short time fourier transform of a signal"""
    n = len(sig)
    nperseg = n // windows
    freqs, times, power = signal.stft(sig, fs=44100, window='boxcar', nperseg=nperseg, noverlap=0, boundary=None)
    if return_all:
        return power, freqs, times
    else:
        return power

def istft(stft_sig, windows = 40, sig_len = 88200, return_times=True):
    nperseg = sig_len // windows
    times, magnitudes = signal.istft(stft_sig, window='boxcar', fs=44100, nperseg = nperseg, noverlap = 0, boundary=False)
    if return_times:
        return magnitudes, times
    else:
        return magnitudes

def db(magnitudes):
    """converts 1d vector of frequency magnitudes to decibel representation"""
    return 10 * np.log10(magnitudes)

def mag(decibels):
    """converts decibel vector to magnitudes"""
    return (10 ** (decibels/10))