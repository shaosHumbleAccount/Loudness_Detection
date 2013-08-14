#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Scope:
    LatestData = 0
    def __init__(self, ax, maxt=2, dt=0.02):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-.1, 1.1)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt: # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,




def getLatestData():
    print Scope.LatestData
    yield Scope.LatestData
    
def callback(data):
    Scope.LatestData = data.data

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Loudness", Float32, callback)
    

if __name__ == '__main__':
    listener()
    fig, ax = plt.subplots()
    scope = Scope(ax)

    # pass a generator in "emitter" to produce data for the update func
    ani = animation.FuncAnimation(fig, scope.update, getLatestData, interval = 30,
                                  blit=True)
    plt.show()