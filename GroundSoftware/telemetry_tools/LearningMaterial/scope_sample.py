#!/usr/bin/python2
"""
Emulate an oscilloscope.  Requires the animation API introduced in
matplotlib 1.0 SVN.
This example is based partly on http://matplotlib.org/examples/animation/strip_chart_demo.html (with modifications to adapt to streaming data)
"""
# (c) 2016, Abhimanyu Ghosh
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

# A class to plot a stream of data on an animated Matplotlib window. 
# "maxt" essentially sizes the width of the scope plot
# "dt" essentially affects the resolution of the plot in terms of samples/window
# "zoom_step" affects how fast the scroll wheel zooms in/out in time

class Scope(object):
    def __init__(self, ax, maxt=10, dt=0.02, zoom_step=1.5):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-1000, 10000)
        self.ax.set_xlim(0, self.maxt)
        self.zoom_step = zoom_step

    # A function to auto-resize the Y-axis in response to the true range of data from the previous "maxt" worth of data points
    def resize_y(self, vert_buffer=0.2):
        # First get the Y-axis limits at present:
        vert_limits = self.ax.get_ylim()
        ymin = vert_limits[0]
        ymax = vert_limits[1]

        # Counters to track data max and min:
        ydata_max = 0.0
        ydata_min = 0.0

        # Look through the data to find true max and min:
        for i in range(len(self.ydata)):
            if self.ydata[i] > ydata_max:
                ydata_max = self.ydata[i]
            if self.ydata[i] < ydata_min:
                ydata_min = self.ydata[i]

        # Update desired Y-axis limits, and add in a buffer space so points at the edge are more visible:
        ymax = ydata_max + vert_buffer
        ymin = ydata_min - vert_buffer

        # Propagate these changes to the plot:
        self.ax.set_ylim(ymin, ymax)

    def update(self, y):
        # Get the last "x-axis" point in the array for that axis:
        lastt = self.tdata[-1]
        # If we're at the end of a horizontal period (i.e. "maxt" number of points collected)
        # we reset the array elements, recompute Y-axis limits and shift the X-axis forward another "maxt" amount:
        if lastt > self.tdata[0] + self.maxt:
            self.tdata = [self.tdata[-1]]
            self.resize_y()
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        # We keep putting new data into the Y-axis buffer and adding to the X-axis:
        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,

    # A callback for Matplotlib to go to when the user scrolls in/out while having cursor over the plot area:
    def time_zoom_handler(self, event):
        # A simple exponential zoom. event.step determines if the user is scrolling in/out 
        # and thus the direction of our zooming action:
        zoom_multiplier = 1.0
        zoom_multiplier *= math.exp(event.step*self.zoom_step)

        self.maxt *= zoom_multiplier

        # If the user is zooming out, we want to trigger a re-draw so we're not waiting forever 
        # for a whole new set of data... just "stretch" the existing data to fit the new range
        # and resize the X-axis appropriately:

        if self.tdata[-1] < self.tdata[0] + self.maxt:
            self.line.set_data(self.tdata, self.ydata)
            self.ax.set_xlim(self.tdata[0], self.tdata[-1])
            self.ax.figure.canvas.draw()
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)

# A stand-in function to generate the data to be plotted:
def emitter(p=0.03):
    # Sine wave instead...
    t = 0.0
    while True:
        t += 0.01
        yield math.sin(t*10.0)

fig, ax = plt.subplots()
scope = Scope(ax, zoom_step=0.2)
f2 = ax.get_figure()

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter, interval=2, blit=True)

f2.canvas.mpl_connect('scroll_event', scope.time_zoom_handler)


plt.show()