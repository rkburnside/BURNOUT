#pygame draw
import serial
#import pygame
#from pygame.locals import *
from sys import exit
from random import *

ser = serial.Serial(13, 115200, timeout=1000)

while 1:  # make this break when serial times out
    temp = ser.readline()
    if temp[0] = "$":
        temp = temp[1:].split(",")
        for item in temp:
            #print item to window
    else if temp[0] = "#":
        temp = temp[1:].split(",")
        wp = float(temp[0])
        xcoor = float(temp[1])
        ycoor = float(temp[2])
        #plot coordinate on graph, add to databas?
    else if temp[0] = "d":
        temp = temp[1:].split(",")
        for item in temp:
            # some item = float(temp[i]

# print coordinates plot, mark waypoints, draw LOS lines
# possilbly color by speed

# plot: rates for accel and gyro, speed, angle

# save final result to png file for now
#maybe try to picle everything. and save to file.
