#!/usr/bin/python

import rospy
import pyaudio
import struct
import math
from time import sleep
from std_msgs.msg import String, Float32
from threading import RLock

INITIAL_TAP_THRESHOLD = 0.1
FORMAT = pyaudio.paInt16 
SHORT_NORMALIZE = (1.0/32768.0)
CHANNELS = 2
RATE = 44100  
INPUT_BLOCK_TIME = 0.1
INPUT_FRAMES_PER_BLOCK = int(RATE*INPUT_BLOCK_TIME)

def get_rms( block ):
    # RMS amplitude is defined as the square root of the 
    # mean over time of the square of the amplitude.
    # so we need to convert this string of bytes into 
    # a string of 16-bit samples...

    # we will get one short out for each 
    # two chars in the string.
    count = len(block)/2
    format = "%dh"%(count)
    shorts = struct.unpack( format, block )

    # iterate over the block.
    sum_squares = 0.0
    for sample in shorts:
        # sample is a signed short in +/- 32768. 
        # normalize it to 1.0
        n = sample * SHORT_NORMALIZE
        sum_squares += n*n
    return math.sqrt( sum_squares / count )

class Audio_Listener(object):
    def __init__(self):
        self.pa = pyaudio.PyAudio()
        self.stream = self.open_mic_stream()
        self.tap_threshold = INITIAL_TAP_THRESHOLD
        self.latestAmplitude = 0
        self.lock = RLock()
        
    def stop(self):
        self.stream.close()

    def find_input_device(self):
        device_index = None            
        for i in range( self.pa.get_device_count() ):     
            devinfo = self.pa.get_device_info_by_index(i)   
            print( "Device %d: %s"%(i,devinfo["name"]) )

            for keyword in ["mic","input"]:
                if keyword in devinfo["name"].lower():
                    print( "Found an input: device %d - %s"%(i,devinfo["name"]) )
                    device_index = i
                    return device_index

        if device_index == None:
            print( "No preferred input found; using default input device." )

        return device_index

    def open_mic_stream( self ):
        device_index = self.find_input_device()

        stream = self.pa.open(   format = FORMAT,
                                 channels = CHANNELS,
                                 rate = RATE,
                                 input = True,
                                 input_device_index = device_index,
                                 frames_per_buffer = INPUT_FRAMES_PER_BLOCK)

        return stream

    def listen(self):
        with self.lock:
            try:
                block = self.stream.read(INPUT_FRAMES_PER_BLOCK)
            except IOError, e:
                self.errorcount += 1
                print( "(%d) Error recording: %s"%(self.errorcount,e) )
                self.noisycount = 1
                return
            amplitude = get_rms( block )
            self.latestAmplitude = amplitude
            #print amplitude
            if amplitude > self.tap_threshold:
                print "Oh, your loudness is: " + str(amplitude)
                # noisy block
            else:            
                # quiet block.
                pass
            return amplitude
    
    
def audio_listener():
    pub = rospy.Publisher('Loudness', Float32)
    rospy.init_node('audio_listener')
    listener = Audio_Listener()

    # pass a generator in "emitter" to produce data for the update func
    while not rospy.is_shutdown():
        loudness = listener.listen()
        pub.publish(Float32(loudness))
        rospy.sleep(0.01)

    
if __name__ == "__main__":
    audio_listener()
    


    

    