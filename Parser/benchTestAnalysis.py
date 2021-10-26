# -*- coding: utf-8 -*-
"""
Created on Mon Aug  9 07:47:06 2021

@author: SparkyVT
"""

""" Objectives for the bench test analysis:
    
    1)  Check Z-axis is aligned to the sky on the IMU and High-G accelerometers
    2)  Ensure that the High-G accelerometer and IMU are pointed in the same direction
    3)  Check that fusion velocity and altitude are reasonable
    4)  Plot rotation
    5)  Ensure that SD card latency is not extreme
    6)  Check the number of telemetry packets sent
    7)  Check flight events for the appropriate order and correct events
    8)  Plot sensor data to check for anamolies
    9)  Check expected GPS update rate
    10) Ensure the test time matches the expected length
 """
 
"""-------------------------------
IMPORT DATA FILE
-------------------------------"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import win32gui, win32con, os

plt.rcParams['savefig.dpi'] = 300
plt.rcParams["figure.dpi"] = 300

filter='Data Files\0*.txt;\0'
customfilter='*.txt'
path, customfilter, flags=win32gui.GetOpenFileNameW(
    InitialDir=os.environ['temp'],
    Flags=win32con.OFN_ALLOWMULTISELECT|win32con.OFN_EXPLORER,
    #File='somefilename', 
    DefExt='txt',
    Title='Select Rocket Data File',
    Filter=filter,
    CustomFilter=customfilter,
    FilterIndex=0)

df = pd.read_csv(
    path, 
    dtype={'fltEvents': str}, 
    engine='python', 
    delimiter = ",", 
    skiprows = 0, 
    skipfooter= 8, 
    index_col = False)

df.columns.values[0] = "timeStamp"
df['flightTime']= df['timeStamp']/1000000

"""-------------------------------
SET ACCELEROMETER GAINS
-------------------------------"""
teensyGen = 3.2
if teensyGen == 1:
    accelGain = 0.012 #LSM303
    gainHighG = 0.0183 #ADXL377 & ADS1115 Combo
    centerPoint = 13125 #ADXL377 & ADS1115 Combo
    centerPoint = 0
elif teensyGen == 3:
    accelGain = 0.000732 #LSM9DS1
    gainHighG = 1/129 #ADXL377 and Teensy ADC
    highGoffset = 1
    centerPoint = 0
elif teensyGen == 3.1:
    accelGain = 0.012 #LSM303
    gainHighG = 1/129 #ADXL377 and Teensy ADC
    centerPoint = 0
elif teensyGen == 3.2 or teensyGen == 3.3:
    accelGain = 0.000732 #LSM9DS1
    centerPoint = 0
    gainHighG = 0.049 #H3LIS331DL

#Calculate the cycle times    
dataRate = df.loc[:,'timeStamp'].index.max()/15
for i in range(df.timeStamp.index.max()):
    
    if i == 0: df.at[i, 'dt'] = 0
    else: df.at[i, 'dt'] = df.at[i, 'timeStamp'] - df.at[i-1, 'timeStamp']

#plot the cycle times
df['dt'].plot(
    x = 'flightTime',
    xlabel = "Flight Time",
    y = 'dt',
    ylabel = "Cycle Time",
    title = 'Samples per Second: ' + str(dataRate)[0:4])

#plot the accelerometers
df['accelZ']*= accelGain
df['smoothHighGz']*= gainHighG
df['highGz']*= gainHighG
df.plot(
    x = 'flightTime',
    xlabel = "Flight Time",
    y = ['highGz','smoothHighGz','accelZ'],
    ylabel = "G-load",
    title = 'Acceleration Data')

#look at the first second of data
df.head(1000).plot(
    x = 'flightTime',
    xlabel = "Flight Time",
    y = ['smoothHighGz','accelZ'],
    ylabel = "m/s^2",
    title = 'Accleration Data')

#plot rotation
df['pitchX']*=0.1
df['yawY']*=0.1
df['offVert']*=0.1
df.plot(
        x = 'flightTime',
        xlabel = "Flight Time",
        y = ['rollZ', 'yawY', 'pitchX','offVert'],
        ylabel = "Degrees",
        title = 'Rotation Data')

#plot gyro data
df.plot(
        x = 'flightTime',
        xlabel = 'Flight Time',
        y = ['gyroX', 'gyroY', 'gyroZ'],
        ylabel = 'degrees per second',
        title = 'Gyro Output Data')

#plot baro temp
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = 'baroTemp',
    ylabel = 'deg C',
    title = 'Barometric Temperature')

#plot baro data
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = ['baroAlt','altMoveAvg'],
    ylabel = 'meters',
    title = 'Barometric Altitude')

#plot baro velocity
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = ['baroVel'],
    ylabel = 'Baro Velocity',
    title = 'Barometric Velocity')

#plot integrated velocity
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = 'intVel',
    ylabel = 'meter per second',
    title = 'IMU Velocity')

#plot integrated altitude
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = 'intAlt',
    ylabel = 'meters',
    title = 'IMU Altitude')

#plot fusion velocity
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = 'fusionVel',
    ylabel = 'meter per second',
    title = 'Fusion Velocity')

#plot fusion velocity
df.fillna(method='pad').plot(
    x = 'flightTime',
    xlabel = 'Flight Time',
    y = 'fusionAlt',
    ylabel = 'meter per second',
    title = 'Fusion Altitude')

#plot gps packets
df.fillna(method='pad').plot(
        y='gnssSatellites',
        x='flightTime',
        title = 'GNSS Coords RXd')

#plot telemetry packets
for i in range(df.timeStamp.index.max()):
    if pd.isna(df.at[i, 'radioPacketNum']): df.at[i, 'packetNum']=0
    else: df.at[i, 'packetNum']=1
df.plot(
        y='packetNum',
        x='flightTime',
        title = 'Packets Sent',
        legend = False)

