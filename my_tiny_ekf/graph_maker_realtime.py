# -*- coding: utf-8 -*-
from calendar import c
from inspect import _void
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.style as mplstyle
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
# Data Load
from serial import Serial
import serial
import time
import signal
import threading
import ctypes
from ctypes import *
i = c_double(0)
pi = pointer(i)
x = ""
y = ""
z = ""
x_val = []
y_val = []
z_val = []
text = ""

port = 'COM3'
# port = '/dev/cu.usbmodem1301'
baud = 115200
ser = Serial(port,baud) 
exitThread = False


def animate(i):
    if ser.readable():
        res = ser.readline()
        #print(res.decode()[:len(res)-1])
        
        if "Position" in res.decode()[:len(res)-1]:
            text = res.decode()[:len(res)-1]
        #print(text)
            text_splited = text.split(",")
            if len(text_splited) > 1:
                x = text_splited[1]
                y = text_splited[2]
                z = text_splited[3]
                print("X:" + x + ", Y:" + y + ", Z:" + z)
                x_val.append(float(x))
                y_val.append(float(y))
                z_val.append(float(z))
                plt.cla()
                drawplots()
                plt.plot(x_val[len(x_val)-11: len(x_val)-1], y_val[len(y_val)-11: len(y_val)-1], z_val[len(z_val)-11: len(z_val)-1], color = 'g', alpha = 0.9)
        else:
            text = res.decode()[:len(res)-1]
            print(text)

def drawplots():
    n = 3
    x = [-0.5, 0, 0.5]
    zero = np.zeros(n)
    ax.plot(x, zero, zero, color = 'r', alpha = 0.9)
    ax.plot(zero, x, zero, color = 'r', alpha = 0.9)
    ax.plot(zero, zero, x, color = 'r', alpha = 0.9)
    
        
if __name__ == "__main__":
    
    plt.style.use('fivethirtyeight')
    mplstyle.use('fast')
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    n = 3
    x = [-0.5, 0, 0.5]
    zero = np.zeros(n)
    ax.plot(x, zero, zero, color = 'r', alpha = 0.9)
    ax.plot(zero, x, zero, color = 'r', alpha = 0.9)
    ax.plot(zero, zero, x, color = 'r', alpha = 0.9)
    # AXES PROPERTIES]
    ax.set_xlim3d([-0.5, 0.5])
    ax.set_ylim3d([-0.5, 0.5])
    ax.set_zlim3d([-0.5, 0.5])
    ax.set_xlabel('X(t)')
    ax.set_ylabel('Y(t)')
    ax.set_zlabel('Z(t)')
    ax.set_title('IMU Motion Tracking')
    ani = FuncAnimation(plt.gcf(), animate, interval = 10, cache_frame_data=False)

    plt.show()