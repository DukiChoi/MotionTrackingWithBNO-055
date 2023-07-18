import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from mpl_toolkits.mplot3d import Axes3D

plt.style.use('fivethirtyeight')
 
x_val = []
y_val = []
z_val = []

index = count()

fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(111, projection='3d')

def animate(i):
    x_val.append(math.cos(i))
    y_val.append(math.cos(i))
    z_val.append(math.sin(i))
    plt.cla()
    plt.plot(x_val, y_val, z_val)
 
ani = FuncAnimation(plt.gcf(), animate, interval = 10)
 
 
 
plt.tight_layout()
plt.show()
