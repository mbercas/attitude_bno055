#!/usr/bin/env python

""" Read serial data from the BNO055 sensor and displays it in real time.

  - Original code from: https://thepoorengineer.com/en/arduino-python-plot/#python
     + Modified to include multiple subplots 
     + Modified to include the BNO055 sensor calibration flags
     + Identity OS and switch port automatically
     + Tilt calculations
"""

import platform
from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import pandas as pd
import numpy as np


class SerialPlot:
    def __init__(self, serial_port = '/dev/ttyUSB0', serial_baud = 115200, plot_length = 100, data_num_bytes = 2, num_plots = 1):
        self.port = serial_port
        self.baud = serial_baud
        self.plot_max_length = plot_length
        self.data_num_bytes = data_num_bytes
        self.rawData = bytearray(data_num_bytes)
        self.num_plots = num_plots
        self.data = []
        for _ in range(num_plots):
            self.data.append(collections.deque([0] * plot_length, maxlen=plot_length))
        self.isRun = True
        self.is_receiving = False
        self.thread = None
        self.plot_timer = 0
        self.previous_timer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')
        try:
            self.serial_connection = serial.Serial(serial_port, serial_baud, timeout=4)
            print('Connected to ' + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.is_receiving != True:
                time.sleep(0.1)

    def getSerialData(self, frame, lines, rects, line_value_text, line_label, time_text):
        current_timer = time.perf_counter()
        self.plot_timer = int((current_timer - self.previous_timer) * 1000)     # the first reading will be erroneous
        self.previous_timer = current_timer
        time_text.set_text('Plot Interval = ' + str(self.plot_timer) + 'ms')
        
        (gyroX, gyroY, gyroZ,
         accX, accY, accZ,
         magX, magY, magZ,
         sysCal, gyroCal, accCal, magCal,
         ) = struct.unpack('9f4b', self.rawData)
        
        data = [gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ]

        for i in range(6): #self.num_plots):
            self.data[i].append(data[i])    # we get the latest data point and append it to our array
            lines[i].set_data(range(self.plot_max_length), self.data[i])
            line_value_text[i].set_text(f"{line_label[i]} = {data[i]:3.5f}")

        angles = get_angles(accX, accY, accZ)
        for angle, i in enumerate(range(6, 8)): #self.num_plots):
            self.data[i].append(angles[angle])    # we get the latest data point and append it to our array
            lines[i].set_data(range(self.plot_max_length), self.data[i])
            line_value_text[i].set_text(f"{line_label[i]} =  {angles[angle]:3.2f} / {(angles[angle] * 180. / np.pi):4.2f}")

            
        for i, h in enumerate([sysCal, gyroCal, accCal, magCal]):
            rects[i].set_height(h)
        # self.csvData.append(self.data[-1])

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serial_connection.reset_input_buffer()
        while (self.isRun):
            self.serial_connection.readinto(self.rawData)
            self.is_receiving = True
            #print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serial_connection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')

def get_angles(x: float, y: float, z: float) -> "tuple[float, float]":
    """Calculate the theta and phi angles
       Returns: theta, phi in radians
    """
    # normalize
    g = np.array([x, y, z])
    r = np.linalg.norm(np.array([x, y, z]))
    g = g/r

    if np.abs(g[2]) < 0.01:
        return (0,0)
    else:
        #return (np.arctan2(np.linalg.norm(g[:2]), g[2]), np.arctan2(g[0], g[1]))
        return (np.arctan2(np.linalg.norm(g[[0,2]]), g[1]), np.arctan2(g[2], g[0]))


def smooth(prev_value: float, sample: float, coeff: float) -> float:
    """ Basic exponential smoothing """
    return sample * coeff + (1 - coeff) * prev_value;
        

def main():

    running_os = platform.system()
    if running_os == 'Windows':
        portName = 'COM9' 
    else:
        portName = '/dev/ttyUSB0'
    baudRate = 115200
    max_plot_length = 100
    data_num_bytes =  4+3*4*3       # calibration data + 9 floats
    num_plots = 0

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = max_plot_length
    ymin = -(200)
    ymax = 200
    all_line_labels = []

    fig = plt.figure(figsize=(13, 10))
    
    #ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    
    ax = plt.subplot(411, xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Gyro')
    #ax.set_xlabel("time")
    ax.set_ylabel("rad/sec")

    time_text = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    line_value_text = []
    line_label = ['X', 'Y', 'Z']
    for i in range(3):
        lines.append(ax.plot([], [], label=line_label[i])[0])
        line_value_text.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))
        num_plots += 1
    all_line_labels = line_label 
    ymin = -(5)
    ymax = 5

        
    ax = plt.subplot(412, xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Accelerometer')
    #ax.set_xlabel("time")
    ax.set_ylabel("g")

    for i in range(3):
        lines.append(ax.plot([], [], label=line_label[i])[0])
        line_value_text.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))
        num_plots += 1
    all_line_labels += line_label

    
    ymin = -5
    ymax = 5
    ax = plt.subplot(413, xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Tilt')
    ax.set_xlabel("time")
    ax.set_ylabel("rads")

    tilt_label= [r"$\theta$", r"$\phi$"]
    for i in range(2):
        lines.append(ax.plot([], [], label=tilt_label[i])[0])
        line_value_text.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))
        num_plots += 1
        all_line_labels += tilt_label
    plt.legend(loc="upper left")
        
    ax = plt.subplot(414,  ylim=(0, 4))
    rects = plt.bar(range(4), 3, tick_label=['SysCal', 'GyroCal', 'AccCal', 'MagCal'])

    
    s = SerialPlot(portName, baudRate, max_plot_length, data_num_bytes, num_plots)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    anim = animation.FuncAnimation(fig,
                                   s.getSerialData,
                                   fargs=(lines, rects, line_value_text, all_line_labels, time_text),
                                   interval=pltInterval)    # fargs has to be a tuple


    plt.show()

    s.close()


if __name__ == '__main__':
    main()
