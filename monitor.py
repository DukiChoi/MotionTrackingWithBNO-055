import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import math as m
from struct import *
import re
# %%
from mpl_toolkits.mplot3d import Axes3D
# Data Load
import datetime
import signal
import threading
import pandas as pd
x = []
y = []
z = []

now = datetime.datetime.now()
time = now.strftime("%c")
time = re.sub('[-=+,#/\?:^.@*\"※~ㆍ!』‘|\(\)\[\]`\'…》\”\“\’·]', ' ', time)
count = 0
i = 0
ser = serial.Serial()
ser.port = 'COM5' # 아두이노가 연결된 포트
ser.baudrate = 9600 # baudrate를 지정해줄 수 있다.
# ser.timeout = 0.01 #시리얼에 데이터를 불러올 때 지정하는 딜레이

exitThread = False
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ,0],
                   [ 0, m.cos(theta),-m.sin(theta),0],
                    [ 0, m.sin(theta), m.cos(theta),0],
                   [0, 0, 0, 1]])

def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta),0],
                   [ 0           , 1, 0           ,0],
                   [-m.sin(theta), 0, m.cos(theta),0],
                   [0, 0, 0, 1]])
  2
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0,0 ],
                   [ m.sin(theta), m.cos(theta) , 0,0 ],
                   [ 0           , 0            , 1,0 ],
                   [0, 0, 0, 1]])

trans = np.matrix([[(3/2)**(1/2), 0, 0, 0],
                  [0, (3/2)**(1/2), 0, 0],
                  [0,0,1,0],
                  [0,0,0,1]])  
# phi = m.pi/4
# theta =  -((3/2)*m.pi - m.acos(1/(3**(1/2))))
# psi = 0

def func(num, dataSet, line, redDots):
    # NOTE: there is no .set_data() for 3 dim data...
    line.set_data(dataSet[0:2, :num])
    line.set_3d_properties(dataSet[2, :num])
    redDots.set_data(dataSet[0:2, :num])    
    redDots.set_3d_properties(dataSet[2, :num])
    return line

def handler(signum, frame):
    exitThread = True

# 데이터 처리할 함수
def parsing_data(data):
    tmp = ''.join(data)

    print(tmp)

data5 = []
k= []
# 시리얼을 열어준다.
ser.open()

# 본 쓰레드
def readThread(ser):
    global line
    global exitThread
    global x
    global y
    global z

    while not exitThread:
        idx = 0
        for c in ser.read():
            if idx % 3 == 0:
                x.append(float(c))
            elif idx % 3 == 1:
                y.append(float(c))
            else:
                z.append(float(c))
                
            idx = idx + 1

if __name__ == "__main__":
    numDataPoints = 800
    # fig = plt.figure()
    # ax = Axes3D(fig)
    # ax.set_xlim3d([-10, 10])
    # ax.set_ylim3d([-10, 10])
    # ax.set_zlim3d([-10, 10])
    # ax.set_xlabel('X(t)')
    # ax.set_ylabel('Y(t)')
    # ax.set_zlabel('Z(t)')
    # ax.set_title('Trajectory of electron for E vector along [120]')
    dataSet = []
    index = 1
    # R = Rz(psi) * Ry(theta) * Rx(phi)
    for i in range(numDataPoints):
        # data5 = 
        # time.sleep(1)
        try:
            k = unpack('fffc', ser.readline())
            
            list(k)
            # print(k)
            
            count = count +1
            # print(ser.readline())        
            # # print(ser.read(size=1))
            l = [round(k[0],3), round(k[1],3), round(k[2],3
                                                     )]
            # l = [k[0], k[1], k[2]]
            print(l)
            # l = np.array(l)
            # C = np.dot(R, l)
            # print(C[0][0])
            # x.append(k[0])
            # y.append(k[1])
            # z.append(k[2])
            dataSet.append(l)
            # dataSet.append(C[0][0:3])
            # print(count)
            
            

            # GET SOME MATPLOTLIB OBJECTS
            
            
            # redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='r', marker='o')[0]  # For scatter plot
            # # NOTE: Can't pass empty arrays into 3d version of plot()
            # line = plt.plot(k[0], k[1], k[2], lw=2, c='g')[0]  # For line plot
            # plt.pause(0.001)

            # AXES PROPERTIES]
            

            # Creating the Animation object
            # line_ani = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(dataSet, line, redDots), interval=50,
            #                                 blit=False)
            # # line_ani.save(r'Animation.mp4')
            index += 1

            
        except:
            print("error!")
    # print(x)
    # print(y)
    # print(z)
    # plt.close()
    
    # import csv
    # f = open("data2.csv", "w")
    # writer = csv.writer(f)
    # # np.transpose(dataSet)
    # writer.writerows(dataSet) ## 여기 주목!
    
    
    
    # print(dataSet.shape)
    d = pd.DataFrame(dataSet)
    d.to_csv('dd.csv')
    #pd array형태로  csv파일 읽어오기
    #####여기서 txt명과 데이터 개수를 적어주세요#####
    ##############################################
    file_name = "dd"
    ##############################################

    new_df = pd.read_csv('./'+ file_name + '.csv')
    m = new_df.values
    #print(m)
    data_x = m[0:numDataPoints, 1:2].astype(np.float64)
    data_y = m[0:numDataPoints, 2:3].astype(np.float64)
    data_z = m[0:numDataPoints, 3:4].astype(np.float64)

    # data_x = np.array(data_x).flatten().tolist()
    # data_y = np.array(data_y).flatten().tolist()
    # data_z = np.array(data_z).flatten().tolist()
    
    dataSet = np.array([data_x,data_y, data_z]).squeeze(axis=2)
    print(dataSet.shape)
    
    c_array = np.concatenate([data_x, data_y, data_z])
    # print(c_array.shape)
    print("data_x:\n", data_x)
    print("data_y:\n", data_y)
    print("data_z:\n", data_z)
    # print(len(data_x))
    #c_array 값 출력
    # idx =0
    # for c in c_array:
    #     if idx %3 ==0 :
    #         x.append(c)
    #     elif idx%3 ==1:
    #         y.append(c)
    #     elif idx%3 ==2:
    #         z.append(c)
    #     idx = idx + 1
    # type(dataSet)
    # dataSet = np.array([x, y, z]).astype(float)
    # dataSet = c_array
    # numDataPoints = 200
    # b = [[data_x],[data_y],[data_z],[1]]

    # C = np.dot(R, dataSet)
    # list(C)
    # GET SOME MATPLOTLIB OBJECTS
    fig = plt.figure()
    ax = Axes3D(fig)
    # redDots = plt.plot(C[0][0], C[0][1], C[0][2], lw=1, c='r', marker='o')[0]  # For scatter plot
    redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=1, c='b', marker='o')[0]  # For scatter plot
    
    # NOTE: Can't pass empty arrays into 3d version of plot()
    # line = plt.plot(C[0][0], C[0][1], C[0][2], lw=2, c='g')[0]  # For line plot
    line = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=1, c='b')[0]  # For line plot
    0
    # AXES PROPERTIES
    ax.set_xlim3d([-10, 10])               
    ax.set_ylim3d([-10, 10])
    ax.set_zlim3d([-10, 10])
    ax.set_xlabel('X(t)')
    ax.set_ylabel('Y(t)')
    ax.set_zlabel('Z(t)')
    ax.set_title('Trajectory of electron for E vector along [120]')

    # Creating the Animation object7
    
    line_ani = animation.FuncAnimation(fig, func, frames=index, fargs=(dataSet, line, redDots), interval=10,
                                       blit=False)
    # line_ani.save(time+'.gif')

    plt.show()