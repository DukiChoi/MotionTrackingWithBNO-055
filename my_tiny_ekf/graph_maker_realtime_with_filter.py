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
import math
from scipy import signal

x = ""
y = ""
z = ""
x_array = []
y_array = []
z_array = []
x_filtered_arr = []
y_filtered_arr = []
z_filtered_arr = []
text = ""
pi = 0
theta = 0
psi = 0
port = 'COM3'
# port = '/dev/cu.usbmodem21201'
baud = 115200
ser = Serial(port,baud) 
exitThread = False
DEG_2_RAD = 0.01745329251;

def animate(i):
    if ser.readable():
        
        res = ser.readline()
        #print(res.decode()[:len(res)-1])
        if "Position" in res.decode()[:len(res)-1]:
            text = res.decode()[:len(res)-1]
            text_splited = text.split(",")
            if len(text_splited) > 1:
                x = text_splited[1]
                y= text_splited[2]
                z = text_splited[3]
                pi = text_splited[5]  
                theta = text_splited[6] 
                psi = text_splited[7][:-1]
                b,a = signal.butter(12, 0.1, output='ba')

                print("Orientation = (" + pi + ", " + theta + ", " + psi + ")")
                x_array.append(float(x))
                y_array.append(float(y))
                z_array.append(float(z))
                if(len(x_array) > 60):
                    plt.cla()
                    drawplots()
                    x_filtered = signal.filtfilt(b, a, x_array)
                    y_filtered = signal.filtfilt(b, a, y_array)
                    z_filtered = signal.filtfilt(b, a, z_array)
                    print("Position : (" + str(x_filtered[len(x_filtered)-1]) + ", " + str(y_filtered[len(y_filtered)-1]) + ", " + str(z_filtered[len(z_filtered)-1]) + ")")
                    x_filtered_arr.append(x_filtered[len(x_filtered)-1])
                    y_filtered_arr.append(y_filtered[len(y_filtered)-1])
                    z_filtered_arr.append(z_filtered[len(z_filtered)-1])
                    plt.plot(x_filtered_arr[len(x_filtered_arr)-11: len(x_filtered_arr)-1], y_filtered_arr[len(y_filtered_arr)-11: len(y_filtered_arr)-1], z_filtered_arr[len(z_filtered_arr)-11: len(z_filtered_arr)-1], color = 'g', alpha = 0.6)
                    # plt.plot(x_filtered[len(x_filtered)-11: len(x_filtered)-1], y_filtered[len(y_filtered)-11: len(y_filtered)-1], z_filtered[len(z_filtered)-11: len(z_filtered)-1], color = 'g', alpha = 0.6)
                    
                    # print("Position : (" + str(x_array[len(x_array)-1]) + ", " + str(y_array[len(y_array)-1]) + ", " + str(z_array[len(z_array)-1]) + ")")
                    
                    pi = float(pi)*DEG_2_RAD #yaw
                    theta = float(theta)*DEG_2_RAD #pitch
                    psi = float(psi)*DEG_2_RAD #roll
                    

                    
                    # plt.plot(x_array[len(x_array)-11: len(x_array)-1], y_array[len(y_array)-11: len(y_array)-1], z_array[len(z_array)-11: len(z_array)-1], color = 'g', alpha = 0.6)
                    
                    
                    xx = [[x_filtered[len(x_filtered)-1], x_filtered[len(x_filtered)-1] + 0.1, x_filtered[len(x_filtered)-1] + 0.2],
                        [y_filtered[len(y_filtered)-1], y_filtered[len(y_filtered)-1], y_filtered[len(y_filtered)-1]],
                        [z_filtered[len(z_filtered)-1], z_filtered[len(z_filtered)-1], z_filtered[len(z_filtered)-1]]]
                    yy = [[x_filtered[len(x_filtered)-1], x_filtered[len(x_filtered)-1], x_filtered[len(x_filtered)-1]],
                        [y_filtered[len(y_filtered)-1], y_filtered[len(y_filtered)-1] + 0.1, y_filtered[len(y_filtered)-1] + 0.2],
                        [z_filtered[len(z_filtered)-1], z_filtered[len(z_filtered)-1], z_filtered[len(z_filtered)-1]]]
                    zz = [[x_filtered[len(x_filtered)-1], x_filtered[len(x_filtered)-1], x_filtered[len(x_filtered)-1]],
                        [y_filtered[len(y_filtered)-1], y_filtered[len(y_filtered)-1], y_filtered[len(y_filtered)-1]],
                        [z_filtered[len(z_filtered)-1], z_filtered[len(z_filtered)-1] + 0.1, z_filtered[len(z_filtered)-1] + 0.2]]
                    
                    # rotation_matrix = [[math.cos(theta)*math.cos(psi), -math.cos(theta)*math.sin(psi), math.sin(theta)],
                    #       [math.cos(pi)*math.sin(psi)+math.sin(pi)*math.sin(theta)*math.cos(psi), math.cos(pi)*math.cos(psi) - math.sin(pi)*math.sin(theta)*math.sin(psi), - math.sin(pi)*math.cos(theta)],
                    #       [math.sin(pi)*math.sin(psi)-math.cos(pi)*math.sin(theta)*math.cos(psi), math.sin(pi)*math.cos(psi) + math.cos(pi)*math.sin(theta)*math.sin(psi), math.cos(pi)*math.cos(theta)]]
                    
                    #이게 최종
                    rotation_matrix = [[-math.cos(theta) * math.cos(pi), -math.sin(psi) * math.sin(theta) * math.cos(-pi) + math.cos(psi) * math.sin(-pi), math.cos(psi) * math.sin(theta) * math.cos(-pi) + math.sin(psi) * math.sin(-pi)],
                                        [-math.cos(theta) * math.sin(-pi), - math.sin(psi) * math.sin(theta) * math.sin(-pi) - math.cos(psi) * math.cos(-pi), math.cos(psi) * math.sin(theta) * math.sin(-pi) - math.sin(psi) * math.cos(-pi)],
                                        [math.sin(theta), -math.sin(psi) * math.cos(theta), math.cos(psi) * math.cos(theta)]]
                    
                    # rotation_matrix = [[1,0,0],
                    #                    [0,1,0],
                    #                    [0,0,1]]
                    
                    # rotation_matrix = [[-math.sin(psi)*math.sin(pi) + math.cos(theta)*math.cos(pi)*math.cos(psi), math.sin(psi)*math.cos(pi) + math.cos(theta)*math.sin(pi)*math.cos(psi), -math.cos(psi)*math.sin(theta)],
                    #                    [-math.cos(psi)*math.sin(pi) - math.cos(theta)*math.cos(pi)*math.sin(psi), math.cos(psi)*math.cos(theta) - math.cos(theta)*math.sin(pi)*math.sin(psi), math.sin(psi)*math.sin(theta)],
                    #                    [math.sin(theta)*math.cos(pi), math.sin(theta)*math.sin(pi), math.cos(theta)]]
                    
                    rotation_matrix = np.array(rotation_matrix)
                    xx2 = rotation_matrix.dot(np.array(xx))
                    yy2 = rotation_matrix.dot(np.array(yy))
                    zz2 = rotation_matrix.dot(np.array(zz))
                    
                    #이렇게 평행이동을 해주어야 원점에서의 회전이 아니게됨.
                    xx2[0] = xx2[0] + xx[0][0] - xx2[0][0]
                    xx2[1] = xx2[1] + xx[1][0] - xx2[1][0]
                    xx2[2] = xx2[2] + xx[2][0] - xx2[2][0]
                    yy2[0] = yy2[0] + yy[0][0] - yy2[0][0]
                    yy2[1] = yy2[1] + yy[1][0] - yy2[1][0]
                    yy2[2] = yy2[2] + yy[2][0] - yy2[2][0]
                    zz2[0] = zz2[0] + zz[0][0] - zz2[0][0]
                    zz2[1] = zz2[1] + zz[1][0] - zz2[1][0]
                    zz2[2] = zz2[2] + zz[2][0] - zz2[2][0]
                    
                    
                    plt.plot(xx2[0],xx2[1],xx2[2], color = 'red')
                    plt.plot(yy2[0],yy2[1],yy2[2], color = 'orange')
                    plt.plot(zz2[0],zz2[1],zz2[2] , color = 'yellow')
                    plt.title('IMU Motion Tracking\n<Position> X: ' + text_splited[1] + ', Y:' + text_splited[2] + ', Z:' + text_splited[3]+ '\n<Orientation> Yaw:'+ text_splited[5] + '°, Pitch:' + text_splited[6] + '°, Roll:' + text_splited[7][:-1] + '°')

                
        
        else:
            text = res.decode()[:len(res)-1]
            print(text)
            
def drawplots():
    n = 3
    x = [-0.4, 0, 0.4]
    zero = np.zeros(n)
    ax.plot(x, zero, zero, color = 'black', alpha = 0.6)
    ax.plot(zero, x, zero, color = 'black', alpha = 0.6)
    ax.plot(zero, zero, x, color = 'black', alpha = 0.6)
    ax.set_xlabel('X_axis')
    ax.set_ylabel('Y_axis')
    ax.set_zlabel('Z_axis')
    
        
if __name__ == "__main__":
    
    plt.style.use('fivethirtyeight')
    mplstyle.use('fast')
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    # xp = [-0.4, 0, 0.4]
    # zero = np.zeros(3)
    # ax.plot(xp, zero, zero, color = 'r', alpha = 0.6)
    # ax.plot(zero, xp, zero, color = 'r', alpha = 0.6)
    # ax.plot(zero, zero, xp, color = 'r', alpha = 0.6)
    # AXES PROPERTIES]
    ax.set_xlim3d([-0.4, 0.4])
    ax.set_ylim3d([-0.4, 0.4])
    ax.set_zlim3d([-0.4, 0.4])
    ax.set_xlabel('X(t)')
    ax.set_ylabel('Y(t)')
    ax.set_zlabel('Z(t)')
    ax.set_title('IMU Motion Tracking')
    ani = FuncAnimation(plt.gcf(), animate, interval = 10, cache_frame_data=False)

    plt.show()
    
