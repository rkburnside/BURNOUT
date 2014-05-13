#pygame draw
import serial
#import pygame
#from pygame.locals import *
from sys import exit
from random import *
from pylab import *

ser = serial.Serial(28, 115200, timeout=10)

x = array([])
y = array([])
speed = array([])
gyro = array([])
accel = array([])
steer = array([])
angle = array([])
proximity = array([])
temp = 'cool'

while True:  # make this break when serial times out
    temp = ser.readline()
    if len(temp) < 2 :
        break
    if temp[0] == "$":
        temp = temp[1:].split(",")
        for item in temp:
            print item
            #print item to window
    elif temp[0] == "#":
        temp = temp[1:].split(",")
        wp = float(temp[0])
        xcoor = float(temp[1])
        ycoor = float(temp[2])
        #plot coordinate on graph, add to databas?
    elif temp[0] == "d":
        temp = temp[1:].split(",")
        x = append(x, array([float(temp[0])]))
        y = append(y, array([float(temp[1])]))
        speed = append(speed, array([float(temp[2])]))
        gyro = append(gyro, array([float(temp[3])]))
        accel = append(accel, array([float(temp[4])]))
        steer = append(steer, array([float(temp[5])]))
        angle = append(angle, array([float(temp[6])]))
        proximity = append(proximity, array([float(temp[7])]))
ser.close()
dist = arange(0, len(x), 1)
dist = dist *2.33

import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax1 = fig.add_subplot(111)
#t = np.arange(0.01, 10.0, 0.01)
#s1 = np.exp(t)
ax1.plot(dist, accel, 'b-')
ax1.set_xlabel('distance (in)')
# Make the y-axis label and tick labels match the line color.
ax1.set_ylabel('accel', color='b')
for tl in ax1.get_yticklabels():
    tl.set_color('b')


ax2 = ax1.twinx()
#s2 = np.sin(2*np.pi*t)
ax2.plot(dist, gyro, 'r-')
ax2.set_ylabel('gyro', color='r')
for tl in ax2.get_yticklabels():
    tl.set_color('r')
plt.show()

# print coordinates plot, mark waypoints, draw LOS lines
# possilbly color by speed

# plot: rates for accel and gyro, speed, angle

# save final result to png file for now
#maybe try to picle everything. and save to file.
