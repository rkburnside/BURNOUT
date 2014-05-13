#pygame draw
import serial
#import pygame
#from pygame.locals import *
from sys import exit
from random import *
from pylab import*

ser = serial.Serial(32, 115200, timeout=5)

x = array([])
y = array([])
speed = array([])
gyro = array([])
accel = array([])
steer = array([])
angle = array([])
proximity = array([])
temp = 'cool'

while True:
    temp = ser.readline()
    if len(temp) > 2 :
        if temp[0] == "d":
            break

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
accel = -accel
from scipy.signal import wiener
accel = wiener(accel, 21)

# print coordinates plot, mark waypoints, draw LOS lines
# possilbly color by speed

# plot: rates for accel and gyro, speed, angle

# save final result to png file for now
#maybe try to picle everything. and save to file.

from mpl_toolkits.axes_grid1 import host_subplot
import mpl_toolkits.axisartist as AA
import matplotlib.pyplot as plt

if 1:

    host = host_subplot(111, axes_class=AA.Axes)
    plt.subplots_adjust(right=0.75)

    par1 = host.twinx()
    par2 = host.twinx()
    par3 = host.twinx()
    par4 = host.twinx()
    par5 = host.twinx()

    offset = 60
    new_fixed_axis = par2.get_grid_helper().new_fixed_axis
    par2.axis["right"] = new_fixed_axis(loc="right",
                                        axes=par2,
                                        offset=(offset, 0))
    par2.axis["right"].toggle(all=True)

    new_fixed_axis = par3.get_grid_helper().new_fixed_axis
    par3.axis["right"] = new_fixed_axis(loc="right",
                                        axes=par3,
                                        offset=(offset*2, 0))
    par3.axis["right"].toggle(all=True)

    new_fixed_axis = par4.get_grid_helper().new_fixed_axis
    par4.axis["right"] = new_fixed_axis(loc="right",
                                        axes=par4,
                                        offset=(offset*3, 0))
    par4.axis["right"].toggle(all=True)

    new_fixed_axis = par5.get_grid_helper().new_fixed_axis
    par5.axis["right"] = new_fixed_axis(loc="right",
                                        axes=par5,
                                        offset=(offset*4, 0))
    par5.axis["right"].toggle(all=True)


    #host.set_xlim(0, 2)
    #host.set_ylim(0, 2)

    host.set_xlabel("Distance")
    host.set_ylabel("Speed")
    par1.set_ylabel("Turn rate")
    par2.set_ylabel("Lateral Accel")
    par3.set_ylabel("Angle")
    par4.set_ylabel("Steering")
    par5.set_ylabel("Proximity")

    p1, = host.plot(dist, speed, label="Speed")
    p2, = par1.plot(dist, gyro, label="Turn Rate")
    p3, = par2.plot(dist, accel, label="Lateral Accel")
    p4, = par3.plot(dist, angle, label="Angle")
    p5, = par4.plot(dist, steer, label="Steering")
    p6, = par5.plot(dist, proximity, label="Proximity")

    #par1.set_ylim(0, 4)
    #par2.set_ylim(1, 65)

    host.legend()

    host.axis["left"].label.set_color(p1.get_color())
    par1.axis["right"].label.set_color(p2.get_color())
    par2.axis["right"].label.set_color(p3.get_color())
    par3.axis["right"].label.set_color(p4.get_color())
    par4.axis["right"].label.set_color(p5.get_color())
    par5.axis["right"].label.set_color(p6.get_color())

    plt.draw()
    plt.show()

    #plt.savefig("Test")