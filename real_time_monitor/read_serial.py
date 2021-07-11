#!/usr/bin/env python

""" Read serial data from the BNO055 sensor and displays it in real time.

  - Original code from: https://thepoorengineer.com/en/arduino-python-plot/#python
     + Modified to include multiple subplots 
     + Modified to include the BNO055 sensor calibration flags
"""

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import pandas as pd


class serialPlot:
    def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 115200, plotLength = 100, dataNumBytes = 2, numPlots = 1):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.rawData = bytearray(dataNumBytes)
        self.numPlots = numPlots
        self.data = []
        for i in range(numPlots):
            self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self, frame, lines, rects, lineValueText, lineLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        
        (gyroX, gyroY, gyroZ,
         accX, accY, accZ,
         magX, magY, magZ,
         sysCal, gyroCal, accCal, magCal,
         ) = struct.unpack('9f4b', self.rawData)
        
        data = [gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ]

        for i in range(9): #self.numPlots):
            self.data[i].append(data[i])    # we get the latest data point and append it to our array
            lines[i].set_data(range(self.plotMaxLength), self.data[i])
            lineValueText[i].set_text('[' + lineLabel[i%3] + '] = ' + str(data[i]))

        for i, h in enumerate([sysCal, gyroCal, accCal, magCal]):
            rects[i].set_height(h)
        # self.csvData.append(self.data[-1])

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True
            #print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')


def main():
    # portName = 'COM5'     # for windows users
    portName = '/dev/ttyUSB0'
    baudRate = 115200
    maxPlotLength = 100
    dataNumBytes =  4+3*4*3       # calibration data + 9 floats
    numPlots = 9
    s = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = -(200)
    ymax = 200
    style = ['r-', 'y-', 'z-']
    lineLabel = ['X', 'Y', 'Z']

    fig = plt.figure(figsize=(13, 10))
    
    #ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    
    ax = plt.subplot(411, xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Gyro')
    #ax.set_xlabel("time")
    ax.set_ylabel("rad/sec")

    timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []
    for i in range(3):
        lines.append(ax.plot([], [], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))

    ymin = -(5)
    ymax = 5

        
    ax = plt.subplot(412, xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Accelerometer')
    #ax.set_xlabel("time")
    ax.set_ylabel("g")

    for i in range(3):
        lines.append(ax.plot([], [], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))

    ymin = -(50)
    ymax = 50
    
    ax = plt.subplot(413, xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Magnetometer')
    ax.set_xlabel("time")
    ax.set_ylabel("uT")
    
    for i in range(3):
        lines.append(ax.plot([], [], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))

    plt.legend(loc="upper left")
        
    ax = plt.subplot(414,  ylim=(0, 4))
    rects = plt.bar(range(4), 3, tick_label=['SysCal', 'GyroCal', 'AccCal', 'MagCal'])

    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, rects, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple


    plt.show()

    s.close()


if __name__ == '__main__':
    main()
