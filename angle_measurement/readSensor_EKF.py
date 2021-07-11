#!/usr/bin/env python

""" Read serial data from the BNO055 sensor.

  - Original code from: https://thepoorengineer.com/wp-content/uploads/2018/09/PythonIMU_EKF.zip
     + Modified for the BNO055
     + Added calibration data
"""

from threading import Thread
import serial
import time
import struct
import numpy as np
import pandas as pd


class SerialRead:
    def __init__(self, serialPort='/dev/ttyUSB0', serialBaud=38400, dataNumBytes=2, numParams=1):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numParams = numParams
        self.rawData = bytearray(40)   #(numParams * dataNumBytes)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        self.data = np.zeros(numParams)
        self.cal = np.zeros(4)
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            exit()

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self):
        privateData = self.rawData[:]
        fmt = "9f4b"
        self.data = np.array(list(struct.unpack("9f", self.rawData[:36])))
        self.cal = list(struct.unpack("4b", self.rawData[36:40]))
        
        return (self.data, self.cal)

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
