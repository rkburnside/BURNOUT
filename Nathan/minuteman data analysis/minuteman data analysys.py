#-------------------------------------------------------------------------------
# Name:        module2
# Purpose:
#
# Author:      default
#
# Created:     14/05/2014
# Copyright:   (c) default 2014
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import serial
#from sys import exit
from random import *
from pylab import*
from Tkinter import *

def callback():
    print "click!"
    print e.get()

def openserial():
    global ser
    ser = serial.Serial(int(e.get())-1, 115200, timeout=5)

def closeserial():
    ser.close()

class DataSet():
    def __init__(self):
        self.x = array([])
        self.y = array([])
        self.speed = array([])
        self.gyro = array([])
        self.accel = array([])
        self.steer = array([])
        self.angle = array([])
        self.proximity = array([])
        self.dist = array([])

def wait_init():
    while True:
        temp = ser.readline()
        if temp == "initialize" :
            break

def get_data():
    #wait_init();
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
            d.x = append(d.x, array([float(temp[0])]))
            d.y = append(d.y, array([float(temp[1])]))
            d.speed = append(d.speed, array([float(temp[2])]))
            d.gyro = append(d.gyro, array([float(temp[3])]))
            d.accel = append(d.accel, array([float(temp[4])]))
            d.steer = append(d.steer, array([float(temp[5])]))
            d.angle = append(d.angle, array([float(temp[6])]))
            d.proximity = append(d.proximity, array([float(temp[7])]))
    ser.close()
    d.dist = arange(0, len(d.x), 1)
    d.dist = d.dist *2.33
    d.accel = -d.accel

def filter_data():
    import numpy as np
    d.speed = np.convolve(d.speed, np.ones(4)/4, "same")
    d.accel = np.convolve(d.accel, np.ones(10)/10, "same")

# print coordinates plot, mark waypoints, draw LOS lines
# possilbly color by speed

# plot: rates for accel and gyro, speed, angle

# save final result to png file for now
#maybe try to picle everything. and save to file.

def plot_data():

    from mpl_toolkits.axes_grid1 import host_subplot
    import mpl_toolkits.axisartist as AA
    import matplotlib.pyplot as plt

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

    p1, = host.plot(d.dist, d.speed, label="Speed")
    p2, = par1.plot(d.dist, d.gyro, label="Turn Rate")
    p3, = par2.plot(d.dist, d.accel, label="Lateral Accel")
    p4, = par3.plot(d.dist, d.angle, label="Angle")
    p5, = par4.plot(d.dist, d.steer, label="Steering")
    p6, = par5.plot(d.dist, d.proximity, label="Proximity")

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

def save_data():
    import pickle
    from tkFileDialog import asksaveasfilename
    filename = asksaveasfilename()
    pickle.dump(d, open(filename, "wb" ))

def load_data():
    import pickle
    from tkFileDialog import askopenfilename
    filename = askopenfilename()
    global d
    d = pickle.load(open(filename, "rb"))

if __name__ == '__main__':
    master = Tk()
    b = Button(master, text="Connect", command=openserial)
    b.pack()
    e = Entry(master)
    e.pack()
    e.delete(0, END)
    e.insert(0, "32")
    c = Button(master, text="Get Data", command=get_data)
    c.pack()

    f = Button(master, text="Close Port", command=closeserial)
    f.pack()

    g = Button(master, text="Plot Data", command=plot_data)
    g.pack()

    h = Button(master, text="Save Data", command=save_data)
    h.pack()

    i = Button(master, text="Load Data", command=load_data)
    i.pack()

    j = Button(master, text="Filter Data", command=filter_data)
    j.pack()



    temp = None
    d = DataSet()
    #wait_init()


    mainloop()

 #   main()
