#!/usr/bin/python

import rospy
import pyaudio
import struct
import math
from time import sleep
from std_msgs.msg import String, Float32
from threading import RLock
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from audio_interface.cfg import audio_interfaceConfig


FORMAT = pyaudio.paInt16 
SHORT_NORMALIZE = (1.0/32768.0)
CHANNELS = 2
RATE = 44100  
INPUT_BLOCK_TIME = 0.1
INPUT_FRAMES_PER_BLOCK = int(RATE*INPUT_BLOCK_TIME)

def get_rms( block ):
    """ RMS amplitude is defined as the square root of the 
     mean over time of the square of the amplitude.
     so we need to convert this string of bytes into 
     a string of 16-bit samples...
     we will get one short out for each 
     two chars in the string."""
     
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
    LOUDNESS_THRESHOLD = 0.1
    lock = RLock()
    
    def __init__(self):
        self.pa = pyaudio.PyAudio()
        self.stream = self.open_mic_stream()
        self.latestAmplitude = 0
        
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
        with Audio_Listener.lock:
            try:
                block = self.stream.read(INPUT_FRAMES_PER_BLOCK)
            except IOError, e:
                print( "Error recording: %s"%(e,) )
                return
            amplitude = get_rms( block )
            self.latestAmplitude = amplitude
            #print amplitude
            if amplitude > Audio_Listener.LOUDNESS_THRESHOLD:
                print "Oh, your loudness is: " + str(amplitude)
                # noisy block
            else:            
                # quiet block.
                pass
            return amplitude
    
    
def reconfig_callback(config, level):
    with Audio_Listener.lock:
        print "New Threshould: " + str(config.loudness_threshold)
        Audio_Listener.LOUDNESS_THRESHOLD = config.loudness_threshold
        return config

def audio_listener():
    pub = rospy.Publisher('Loudness', Float32)
    listener = Audio_Listener()
    #keep publishing the loudness, and push alarm when over the threshold
    while not rospy.is_shutdown():
        loudness = listener.listen()
        pub.publish(Float32(loudness))
        rospy.sleep(0.01)



if __name__ == "__main__":
    rospy.init_node("audio_interface", anonymous = False)
    srv = DynamicReconfigureServer(audio_interfaceConfig, reconfig_callback)
    audio_listener()
    


    

    