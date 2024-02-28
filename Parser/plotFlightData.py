# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 19:56:58 2021

@author: Sparky
"""

"""-------------------------------
IMPORT DATA FILE
-------------------------------"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import win32gui, win32con, os
import math as math

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

teensyGen = 3.3
if teensyGen == 1:
    accelGain = 0.012 #LSM303
    gainHighG = 0.0183 #ADXL377 & ADS1115 Combo
    centerPoint = 13125 #ADXL377 & ADS1115 Combo
    centerPoint = 0
    gyroGain = 0.07
elif teensyGen == 3:
    accelGain = 0.000732 #LSM9DS1
    gainHighG = 1/129 #ADXL377 and Teensy ADC
    highGoffset = 1
    centerPoint = 0
    gyroGain = 0.07
elif teensyGen == 3.1:
    accelGain = 0.012 #LSM303
    gainHighG = 1/129 #ADXL377 and Teensy ADC
    centerPoint = 0
    gyroGain = 0.07
elif teensyGen == 2 or teensyGen == 3.2 or teensyGen == 3.3:
    accelGain = 0.000732 #LSM9DS1
    centerPoint = 0
    gainHighG = 0.049 #H3LIS331DL
    gyroGain = 0.07
elif teensyGen == 4:
    accelGain = 1/2048
    gainHighG = .12395
    centerPoint = 0
    gyroGain = 1/16.4

if 'Alt' in df.columns: df.rename(columns = {'Alt' : 'baroAlt'}, inplace=True)
"""-------------------------------
CALCULATE BAROMETRIC VELOCITY
-------------------------------"""
'''check to see if barometric velocity was calculated onboard'''
if not 'baroVel' in df.columns:
    lastVal = len(df.index)
    df.at[0, 'baroVel'] = 0
    arrayPosn = 1
    maxArray = 2
    altArray = np.zeros(maxArray)
    timeArray = np.zeros(maxArray)
    smoothAltArray = np.zeros(maxArray)
    altSum = 0
    filterFull = False
    
    for j in range(lastVal):
        if pd.notnull(df.at[j, 'baroAlt']):
            altSum -= altArray[arrayPosn]
            altSum += df.at[j, 'baroAlt']
            altArray[arrayPosn] = df.at[j, 'baroAlt']
            
            if filterFull==True: 
                smoothAlt = altSum / maxArray
                baroVel = (smoothAlt - smoothAltArray[arrayPosn])/((df.at[j, 'timeStamp']-timeArray[arrayPosn])/1000000)
                smoothAltArray[arrayPosn] = smoothAlt
                
            else: 
                smoothAlt = altSum / (arrayPosn+1)
                baroVel = (smoothAlt - smoothAltArray[0])/((df.at[j, 'timeStamp']-timeArray[0])/1000000)
                smoothAltArray[arrayPosn] = smoothAlt
        
            df.at[j, 'baroVel'] = baroVel
            df.at[j, 'altMoveAvg'] = smoothAlt
            timeArray[arrayPosn] = df.at[j, 'timeStamp']
            arrayPosn += 1
            if arrayPosn >= maxArray: 
                arrayPosn = 0
                filterFull = True

"""CREATE VARIABLES AND IDENTIFY KEY EVENTS"""  
df.columns.values[0] = "timeStamp"
df['flightTime']=df['timeStamp']/1000000
df.loc[:, 'baroVel2'] = df.loc[:, 'baroVel'].fillna(method='pad')
df.loc[:, 'baroAlt2'] = df.loc[:, 'baroAlt'].fillna(method='pad')
if 'aX' in df.columns: df.rename(columns = {'aX' : 'accelZ'}, inplace=True)
if 'aY' in df.columns: df.rename(columns = {'aY' : 'accelY'}, inplace=True)
if 'aZ' in df.columns: df.rename(columns = {'aZ' : 'accelX'}, inplace=True)
if 'gX' in df.columns: df.rename(columns = {'gX' : 'gyroZ'}, inplace=True)
if 'gY' in df.columns: df.rename(columns = {'gY' : 'gyroY'}, inplace=True)
if 'gZ' in df.columns: 
    df.rename(columns = {'gZ' : 'gyroX'}, inplace=True)
    df['gyroX']*= -1
if 'vel' in df.columns: df.rename(columns = {'vel' : 'intVel'}, inplace=True)
if 'alt' in df.columns: df.rename(columns = {'alt' : 'intAlt'}, inplace=True)
if 'analog' in df.columns: 
    df.rename(columns = {'analog' : 'smoothHighGz'}, inplace=True)
    df['highGz'] = df['smoothHighGz']
if 'gps_alt' in df.columns: df.rename(columns = {'gps_alt' : 'gpsAlt'}, inplace=True)
if 'pitch' in df.columns: df.rename(columns = {'pitch' : 'pitchX'}, inplace=True)
if 'yaw'   in df.columns: df.rename(columns = {'yaw' : 'yawY'}, inplace=True)
if 'roll'  in df.columns: df.rename(columns = {'roll' : 'rollZ'}, inplace=True)
if 'rotnX' in df.columns: df.rename(columns = {'rotnX' : 'rollZ'}, inplace=True)
if 'rotnY'   in df.columns: df.rename(columns = {'rotnY' : 'yawY'}, inplace=True)
if 'rotnZ'  in df.columns: df.rename(columns = {'rotnZ' : 'pitchX'}, inplace=True)
if 'gpsAlt'  in df.columns: df.rename(columns = {'gpsAlt' : 'gnssAlt'}, inplace=True)
if 'lat'  in df.columns: df.rename(columns = {'lat' : 'gpsLat'}, inplace=True)
if 'lon'  in df.columns: df.rename(columns = {'lon' : 'gpsLon'}, inplace=True)
if 'gpsLat'  in df.columns: df.rename(columns = {'gpsLat' : 'gnssLat'}, inplace=True)
if 'gpsLon'  in df.columns: df.rename(columns = {'gpsLon' : 'gnssLon'}, inplace=True)
if 'events'  in df.columns: 
    df.rename(columns = {'events' : 'fltEvents'}, inplace=True)
    df.loc[:,'fltEvents']=df.loc[:,'fltEvents'].astype('str')
apogee = df[df['fltEvents'].str[4] == "1"].index[0]
if apogee >= df.index.max()-10: apogee = df[df['fltEvents'].str[4] == "1"].index[0]
burnout = df[df['fltEvents'].str[1:3] == "10"].index[0]

"""-------------------------------
ACCELERATION PLOT
-------------------------------"""
#highGoffset = 0.75
highGoffset = 0
accel = df.loc[:,['timeStamp','accelX','accelY','accelZ','highGz','smoothHighGz']]
accel['accelX_Conv'] = accel['accelX'] * accelGain
accel['accelY_Conv'] = accel['accelY'] * accelGain
accel['accelZ_Conv'] = accel['accelZ'] * accelGain
accel['highGz_Conv'] = (accel['smoothHighGz'] - centerPoint)* gainHighG - highGoffset
accel['flightTime'] = accel['timeStamp']/1000000
gph = accel.head(burnout).plot(
    x = 'flightTime', 
    xlabel = "Flight Time",
    y = ['accelX_Conv', 'accelY_Conv', 'accelZ_Conv', 'highGz_Conv'],
    ylabel = "Acceleration (G-load)",
    title = "Booster Motor Thrust",
    label=['accelX','accelY','accelZ','HighG'])
#gph.set_ylim(-5, 15)
accel.head(apogee).plot(
    x = 'flightTime', 
    xlabel = "Flight Time",
    y = [ 'highGz_Conv', 'accelZ_Conv'],
    ylabel = "Acceleration (G-load)",
    title = "Booster Motor Thrust",
    label=['HighG','accelZ'])
plotData = np.asarray(accel)

#Accelerometer comparison
stopPosn = apogee + 4000
for i in range(stopPosn):
    if i == 0:
        accel.at[i, 'accelVel'] = 0
        accel.at[i, 'highGvel'] = 0
    
    else:
        accel.at[i, 'accelVel'] = accel.at[i-1, 'accelVel'] + (accel.at[i, 'accelZ_Conv'] - 1) * 9.80665 * (accel.at[i, 'timeStamp'] - accel.at[i-1, 'timeStamp'])/1000000
        accel.at[i, 'highGvel'] = accel.at[i-1, 'highGvel'] + (accel.at[i, 'highGz']*0.049 - 1) * 9.80665 * (accel.at[i, 'timeStamp'] - accel.at[i-1, 'timeStamp'])/1000000                                          
if teensyGen != 1 and teensyGen != 3 and teensyGen != 3.1:
    accel.loc[0:stopPosn].plot(
        x = 'flightTime',
        xlabel = "Flight Time",
        y = ['accelVel','highGvel'],
        ylabel = "Velocity m/s",
        title = "Integrated Acceleration: LSM9DS1 vs H3LIS331DL",
        label=['LSM9DS1', 'H3LIS331DL'])

accel.at[apogee, 'highGvel']
accel.at[apogee, 'accelVel']

# #plot for acceleration near apogee
# gph = accel.loc[apogee - 7000:apogee].plot(
#     x = 'flightTime', 
#     xlabel = "Flight Time",
#     y = ['accelX_Conv', 'accelY_Conv', 'accelZ_Conv', 'highGz_Conv'],
#     ylabel = "Acceleration (m/s2)",
#     title = "Sustainer Motor Thrust",
#     label=['accelX','accelY','accelZ','HighG'])
# plotData = np.asarray(accel)
# gph.set_ylim(-1, 1)

# =============================================================================
# #plot for 2nd stage motor
# gph = accel.loc[4350:10800].plot(
#     x = 'flightTime', 
#     xlabel = "Flight Time",
#     y = ['accelX_Conv', 'accelY_Conv', 'accelZ_Conv', 'highGz_Conv'],
#     ylabel = "Acceleration (m/s2)",
#     title = "Sustainer Motor Thrust",
#     label=['accelX','accelY','accelZ','HighG'])
# #plotData = np.asarray(accel)
# gph.set_ylim(-2, 8)
# =============================================================================

"""-------------------------------
FUSION VELOCITY PLOT
-------------------------------"""
#vel = df.loc[:, ['timeStamp','accelZ','intVel','fusionVel','baroVel']]         
vel = df.loc[:, ['timeStamp','accelZ','intVel','baroVel','altMoveAvg','intAlt','baroAlt','gnssAlt','fusionVel']]
vel['flightTime'] = vel['timeStamp']/1000000
peakBaroVel = 0
lastGNSSposn = 0
for i in range(apogee):
    
    if vel.at[i, 'baroVel'] > peakBaroVel: peakBaroVel = vel.at[i, 'baroVel']

    if i == 0:
        vel.at[i,'fusionVel2'] = 0
        
    else: vel.at[i, 'fusionVel2'] = vel.at[i-1, 'fusionVel2'] + (vel.at[i, 'accelZ']*accelGain - 1) * 9.80665 * (vel.at[i, 'flightTime'] - vel.at[i-1, 'flightTime'])
    #if pd.notnull(vel.at[i, 'baroVel']) and vel.at[i, 'baroVel'] < vel.at[i, 'fusionVel2'] and vel.at[i, 'fusionVel2'] < 300 and vel.at[i, 'baroVel'] < peakBaroVel and vel.at[i, 'accelZ'] * accelGain < 0.2:
    if pd.notnull(vel.at[i, 'baroVel']) and vel.at[i, 'fusionVel2'] < 300 and vel.at[i, 'baroVel'] < peakBaroVel and vel.at[i, 'accelZ'] * accelGain < 0.2:
        vel.at[i, 'fusionVel2'] = vel.at[i, 'fusionVel2'] * 0.99 + vel.at[i, 'baroVel'] * 0.01
    
    if i>0 and vel.at[i, 'gnssAlt'] !=  vel.at[i-1, 'gnssAlt']: lastGNSSposn = i

for i in range(apogee, df.index.max()):
    
    if pd.notnull(vel.at[i, 'baroVel']): vel.at[i, 'fusionVel2'] = vel.at[i, 'baroVel']
    else: vel.at[i, 'fusionVel2'] = vel.at[i-1,'fusionVel2']
    
    
    if vel.at[i, 'gnssAlt'] !=  vel.at[i-1, 'gnssAlt']:
        vel.at[i, 'fusionVel2'] = (vel.at[i, 'gnssAlt']-vel.at[lastGNSSposn, 'gnssAlt'])/(vel.at[i, 'flightTime'] - vel.at[lastGNSSposn, 'flightTime']) * 1 + vel.at[i-1, 'fusionVel2'] * 0.0
        lastGNSSposn = i

'''plot velocity up until apogee'''
gph = vel.fillna(method='pad').head(apogee).plot(
    x = 'flightTime', 
    y=['intVel', 'baroVel','fusionVel', 'fusionVel2'], 
    ylabel = "Velocity (m/s)",
    title="Velocity Until Apogee")
#gph.set_ylim(0, 400)
'''plot velocity for the last few seconds before apogee'''
vel.loc[apogee-8000:apogee,:].fillna(method='pad').plot(
    x = 'flightTime', 
    y = ['intVel', 'baroVel', 'fusionVel2', 'fusionVel'], 
    ylabel = "Velocity (m/s)",
    title="Velocity Near Apogee")
'''plot velocity through the entire flight'''
vel.fillna(method='pad').plot(
    x = 'flightTime', 
    y=['intVel', 'baroVel', 'fusionVel2'], 
    ylabel = "Velocity (m/s)",
    title = "Velocity To Touchdown")
vel.loc[:,'intVel'].max()
    
"""-------------------------------
FUSION ALTITUDE PLOT
-------------------------------"""
if df.at[0, 'gnssAlt'] != 0: df['gnssAlt'] -= df.at[0, 'gnssAlt']
alt = df.loc[:, ['timeStamp','accelZ','baroAlt','altMoveAvg','baroVel','intVel','intAlt','gnssAlt','fusionVel','fusionAlt']]
alt['flightTime'] = alt['timeStamp']/1000000     

for i in range(apogee):
    
    if i > 0: dt = (alt.at[i, 'timeStamp'] - alt.at[i-1, 'timeStamp'])/1000000
    
    """Update Velocity"""
    if i ==0: alt.at[i, 'fusionVel2'] = 0    
    else: alt.at[i, 'fusionVel2'] = alt.at[i-1, 'fusionVel2'] + (alt.at[i, 'accelZ']*accelGain - 1) * 9.80665 * dt
    
    if pd.notnull(alt.at[i, 'baroVel']) and alt.at[i, 'fusionVel2'] < 300 and i > 0 and alt.at[i, 'accelZ'] * accelGain < 0.2:
        alt.at[i, 'fusionVel2'] = (alt.at[i-1, 'fusionVel2'] + (alt.at[i, 'accelZ']*accelGain - 1) * 9.80665 * dt) * 0.99 + alt.at[i, 'baroVel'] * 0.01
    
    """Update Altitude"""
    if i == 0: alt.at[i, 'fusionAlt2'] = 0
    else: alt.at[i, 'fusionAlt2'] = alt.at[i-1, 'fusionAlt2'] +  alt.at[i, 'fusionVel2'] * dt

    if pd.notnull(alt.at[i, 'altMoveAvg']) and i > 0: alt.at[i, 'fusionAlt2'] = alt.at[i-1, 'fusionAlt2'] * 0.95 + alt.at[i, 'baroAlt'] * 0.05

# """-------------------------------
# ADJUSTED INTEGRATED ALTITUDE PLOT
# -------------------------------"""
dt = 0
alt.at[0,'accelAlt2'] = 0
alt.at[0,'accelVel2'] = 0

for i in range(1, apogee):
    
    dt = (alt.at[i, 'timeStamp'] - alt.at[i-1, 'timeStamp'])/1000000
    
    """Method #1"""
    #velocity: velNew = velOld + (accelZ * cos(offVert) - 1 ) * dt
    #alt.at[i, 'accelVel2' ] = alt.at[i-1, 'accelVel2'] + ((df.at[i, 'accelZ'] * accelGain) * math.cos(df.at[i, 'offVert'] * 0.1 * math.pi / 180) - 1) * 9.80655 * dt
    
    #altitude: altNew = AltOld + velNew * dt * cos(offVert)
    #alt.at[i, 'accelAlt2'] = alt.at[i-1, 'accelAlt2'] + alt.at[i, 'accelVel2'] * dt * math.cos(df.at[i, 'offVert'] * 0.1 * math.pi / 180)
    
    """Method #2"""
    #formula 2: VelNew = VelPrev + (accel * cos(offVert) - 1) * cos(offVert)
    alt.at[i, 'accelVel2'] = alt.at[i-1, 'accelVel2'] + ((df.at[i, 'accelZ']) * accelGain * math.cos(df.at[i, 'offVert'] * 0.1 * math.pi / 180) - 1) * 9.80655 * dt
    
    #formula 2: altNew = altOld + velNew * dt
    alt.at[i, 'accelAlt2'] = alt.at[i-1, 'accelAlt2'] + alt.at[i, 'accelVel2'] * dt

'''plot altitude up until apogee'''
alt.fillna(method='pad').head(apogee).plot(
    x = 'flightTime', 
    y = ['baroAlt', 'altMoveAvg', 'fusionAlt', 'intAlt', 'gnssAlt'],
    ylabel = 'Altitude (m)',
    xlabel = 'Flight Time',
    label = ['Raw Baro Alt', 'Smooth Baro Alt', 'Sensor Fusion', 'IMU Alt','SatNav Alt'],
    title = 'Altitude Until Apogee')
'''plot altitude for the last few seconds up until apogee'''
alt.loc[apogee-2000:apogee,:].fillna(method='pad').plot(
    x = 'flightTime', 
    #y = ['baroAlt', 'altMoveAvg', 'fusionAlt', 'accelAlt2', 'intAlt'],
    y = ['baroAlt', 'altMoveAvg', 'fusionAlt', 'fusionAlt2'],
    ylabel = "Altitude (m)",
    #label = ['Raw Baro Alt', 'Smooth Baro Alt', 'Sensor Fusion', 'accelAlt2', 'intAlt'],
    label = ['Raw Baro Alt', 'Smooth Baro Alt', 'Sensor Fusion', 'Sensor Fusion2'],
    title='Altitude Near Apogee')
'''plot altitude through the entire flight'''
#data rate
dataRate = apogee/alt.flightTime[apogee]
alt.fillna(method='pad').plot(
    x = 'flightTime', 
    y = ['baroAlt', 'altMoveAvg', 'fusionAlt', 'fusionAlt2','intAlt', 'gnssAlt'],
    label = ['Raw Baro Alt', 'Smooth Baro Alt', 'Onboard Fusion', 'Sensor Fusion2','IMU Alt','SatNav Alt'],
    ylabel = 'Altitude (m)',
    title = "Altitude: Liftoff to Touchdown, Data Rate: " + str(dataRate)[0:4] +' SPS')

# """-------------------------------
# GNSS PLOT
# -------------------------------"""
# df.loc[:, 'latitude']=df.loc[:,'gnssLat'].str[1:10].astype('float32')
# df.loc[:, 'longitude']=df.loc[:,'gnssLon'].str[1:10].astype('float32')
# df['gnssAlt']=df['gnssAlt'].fillna(method ='pad')
# df['latitude'] = df['latitude'].fillna(method ='pad')
# df['longitude'] = df['longitude'].fillna(method ='pad')
# df['latitude'] -= df.at[0, 'latitude']
# df['longitude'] -= df.at[0, 'longitude']
# df['latitude'] *= 1.15*5280
# df['longitude'] *= 0.91*5280
# gph = df.plot(
#     x = 'latitude',
#     y = 'longitude',
#     title = 'Ground Track from Pad (ft)',
#     legend = False,
#     ylabel = 'South-North',
#     xlabel = 'West-East').annotate("Apogee",(df.at[apogee, 'latitude'], df.at[apogee, 'longitude']))

# altRange = (df['gnssAlt'].max() - df['gnssAlt'].min())*3.2808

# gph = plt.figure(figsize=(15,8)).gca(projection='3d')
# gph.plot(df['latitude'], df['longitude'], df['gnssAlt']*3.2808, "b.--")
# gph.set_xlim(-1*altRange/2, altRange/2)
# gph.set_ylim(-1*altRange/2, altRange/2)
# gph.set_zlim(df['gnssAlt'].min()*3.2808, df['gnssAlt'].max()*3.2808)
# gph.set_title('Flight Progression', fontsize= 20)
# gph.set_xlabel('West-East', fontsize = 14)
# gph.set_ylabel('South-North', fontsize = 14)
# gph.set_zlabel('Altitude (ft)', fontsize = 14)

# df['gnssSatellites'] = df['gnssSatellites'].fillna(method = 'pad')
# df.plot(
#     x = 'flightTime',
#     y = 'gnssSatellites',
#     title = 'Satellites Used for Fix',
#     xlabel = 'Flight Time',
#     legend = False)

"""-------------------------------
GYRO PLOT
-------------------------------"""
gyro = df.loc[:, ['timeStamp', 'gyroX', 'gyroY', 'gyroZ']]
gyro['flightTime'] = gyro['timeStamp']/1000000
gyro['gyroXdps'] = gyro['gyroX']*gyroGain
gyro['gyroYdps'] = gyro['gyroY']*gyroGain
gyro['gyroZdps'] = gyro['gyroZ']*gyroGain
#gyro.head(apogee).plot(x = 'flightTime', y = ['gyroZdps', 'gyroYdps', 'gyroXdps'], ylabel = "Degrees per Second")
#gyro.head(apogee).plot(x = 'flightTime', y = ['gyroYdps', 'gyroXdps'], ylabel = "Degrees per Second")


plot1 = gyro.head(apogee).plot(
    x = 'flightTime', 
    y = ['gyroYdps', 'gyroXdps'], 
    ylabel = "Degrees per Second",
    label = ['gyroX','gyroY'])
gyro.head(apogee).plot(
    title = "Rotational Velocity",
    x = 'flightTime', 
    y = 'gyroZdps', 
    xlabel = "Flight Time",
    ylabel = "Degrees per Second",
    secondary_y = True, 
    label = 'gyroZ',
    ax = plot1)

"""-------------------------------
ROTATION PLOT (ONBOARD CALCULATIONS)
-------------------------------"""
rotn = df.loc[:, ['timeStamp', 'rollZ', 'pitchX', 'yawY', 'offVert']]
rotn['flightTime'] = rotn['timeStamp']/1000000
rotn['pitch_Conv'] = rotn['pitchX']/10
rotn['yaw_Conv'] = rotn['yawY']/10
rotn['offVert_Conv'] = rotn['offVert']/10
rotn.head(apogee).plot(
    x = 'flightTime', 
    y = ['pitch_Conv', 'yaw_Conv', 'offVert_Conv'], 
    ylabel = 'Onboard Rotation to Apogee(degrees)',
    xlabel = 'Flight Time',
    label = ['pitchX', 'yawY','Off Vertical'])
rotn.plot(
    x = 'flightTime', 
    y = ['pitch_Conv', 'yaw_Conv', 'offVert_Conv'], 
    ylabel = 'Onboard Rotation to Touchdown(degrees)',
    xlabel = 'Flight Time',
    label = ['pitchX', 'yawY','Off Vertical'])
"""-------------------------
ORIGINAL DCM2D METHOD
---------------------------"""
import math as math
'''Initialize the dataframe'''
origRotn = df.loc[:, ['timeStamp','gyroX','gyroY','gyroZ']]
origRotn['flightTime'] = origRotn['timeStamp']/1000000

'''Initialize the pitch and yaw'''
deg2rad = math.pi / 180
rotn2rad = gyroGain * deg2rad / 1000000
offset = 0

origRotn.at[0, 'dt'] = 0
origRotn.at[0, 'dx'] = df.at[0, 'pitchX'] * 0.1 * 1000000 / gyroGain
origRotn.at[0, 'dy'] = df.at[0, 'yawY']   * 0.1 * 1000000 / gyroGain
origRotn.at[0, 'dz'] = 0

for i in range(1,apogee,1):

  """Calculate dt"""
  origRotn.at[i, 'dt'] = origRotn.at[i, 'timeStamp'] - origRotn.at[i-1, 'timeStamp']
  
  """Calculate new Roll angle from gyro data"""
  origRotn.at[i,'dz'] = origRotn.at[i-1, 'dz'] + origRotn.at[i, 'gyroZ'] * origRotn.at[i, 'dt']

  """calculate the partial rotation angles"""  
  sinZ = math.sin(origRotn.at[i, 'dz'] * rotn2rad)
  cosZ = math.cos(origRotn.at[i, 'dz'] * rotn2rad)
      
  """Compute the new y and x angles"""
  origRotn.at[i, 'dx'] = origRotn.at[i-1, 'dx'] + (cosZ * origRotn.at[i, 'gyroX'] - sinZ * origRotn.at[i, 'gyroY']) * origRotn.at[i, 'dt']
  origRotn.at[i, 'dy'] = origRotn.at[i-1, 'dy'] + (cosZ * origRotn.at[i, 'gyroY'] + sinZ * origRotn.at[i, 'gyroX']) * origRotn.at[i, 'dt']
  
  """Compute Pitch, Yaw, Roll"""
  origRotn.at[i, 'rollZ']  = origRotn.at[i, 'dz'] * gyroGain / 1000000
  origRotn.at[i, 'pitchX'] = origRotn.at[i, 'dx'] * gyroGain / 1000000
  origRotn.at[i, 'yawY']   = origRotn.at[i, 'dy'] * gyroGain / 1000000
    
  """Calculate off vertical"""
  tanYaw = math.tan(origRotn.at[i, 'yawY'] * deg2rad)**2
  tanPitch = math.tan(origRotn.at[i, 'pitchX'] * deg2rad)**2
  origRotn.at[i, 'offVert'] = math.atan(pow(tanYaw + tanPitch, 0.5))/deg2rad

origRotn.head(apogee).plot(x = 'flightTime', y = ['pitchX', 'yawY', 'offVert'], ylabel = "DCM2D Off-Vertical Rotation", xlabel="Flight Time")
origRotn.loc[apogee, 'offVert']
origRotn.head(apogee).plot(x = 'flightTime', y = ['rollZ'])
origRotn.head(apogee).max()

"""-------------------------------
dt plot of time between cycles (indicates possible problems with the SD card)
-------------------------------"""
dtData = df.loc[:, ['timeStamp']]
dtData['flightTime'] = dtData['timeStamp']/1000000
dtData.at[0, 'dt'] = 0
dtData.idxmax()[1]

for i in range(1, dtData.idxmax()[1]):
    dtData.at[i, 'dt'] = dtData.at[i, 'timeStamp'] - dtData.at[i-1, 'timeStamp']
    if dtData.at[i, 'dt'] < 0: dtData.at[i, 'dt'] = 0
    
dtData.head(apogee).plot(x = 'flightTime', y = ['dt'])
dtData.plot(x = 'flightTime', y = ['dt'])

"""-------------------------------
Quaternion Differential Method at intervals with continuous rotation integration
-------------------------------"""
import math as math
'''Initialize the dataframe'''
#quatRotn = df.loc[::15, ['timeStamp','gyroX','gyroY','gyroZ']].reset_index(drop = True)
quatRotn = df.loc[:, ['timeStamp','gyroX','gyroY','gyroZ']]
quatRotn['flightTime'] = quatRotn['timeStamp']/1000000

'''Initialize the quaternion and quaternion differential'''
quatRotn.at[0, 'pitchX'] = df.at[0, 'pitchX'] * 0.1
quatRotn.at[0, 'yawY']   = df.at[0, 'yawY']   * 0.1
quatRotn.at[0, 'rollZ']  = df.at[0, 'rollZ']  * 0.1

quatRotn.at[0, 'Quat1'] = 1
quatRotn.at[0, 'Quat2'] = df.at[0, 'pitchX'] * 0.1 * math.pi / 360
quatRotn.at[0, 'Quat3'] = df.at[0, 'yawY'] * 0.1 *math.pi / 360
quatRotn.at[0, 'Quat4'] = 0
tanYaw = math.tan(df.at[0, 'yawY']*0.1*math.pi/180)
tanYaw = tanYaw * tanYaw
tanPitch = math.tan(df.at[0, 'pitchX']*0.1*math.pi/180)
tanPitch = tanPitch * tanPitch
quatRotn.at[0, 'offVert'] = math.atan(pow(tanYaw + tanPitch, 0.5))*180/math.pi

quatLen = pow(quatRotn.at[0, 'Quat1']**2 + quatRotn.at[0, 'Quat2']**2 + quatRotn.at[0, 'Quat3']**2 + quatRotn.at[0, 'Quat4']**2, -0.5)
quatRotn.at[0, 'Quat1'] *= quatLen
quatRotn.at[0, 'Quat2'] *= quatLen
quatRotn.at[0, 'Quat3'] *= quatLen
quatRotn.at[0, 'Quat4'] *= quatLen

quatRotn.at[0, 'QuatDiff1'] = 0
quatRotn.at[0, 'QuatDiff2'] = 0
quatRotn.at[0, 'QuatDiff3'] = 0
quatRotn.at[0, 'QuatDiff4'] = 0

'''change to 32 bit float since that's the best the flight computer can do'''
quatRotn.loc[:, 'Quat1'] = quatRotn.loc[:, 'Quat1'].astype('float32')
quatRotn.loc[:, 'Quat2'] = quatRotn.loc[:, 'Quat2'].astype('float32')
quatRotn.loc[:, 'Quat3'] = quatRotn.loc[:, 'Quat3'].astype('float32')
quatRotn.loc[:, 'Quat4'] = quatRotn.loc[:, 'Quat4'].astype('float32')
quatRotn.loc[:, 'QuatDiff1'] = quatRotn.loc[:, 'QuatDiff1'].astype('float32')
quatRotn.loc[:, 'QuatDiff2'] = quatRotn.loc[:, 'QuatDiff2'].astype('float32')
quatRotn.loc[:, 'QuatDiff3'] = quatRotn.loc[:, 'QuatDiff3'].astype('float32')
quatRotn.loc[:, 'QuatDiff4'] = quatRotn.loc[:, 'QuatDiff4'].astype('float32')

deg2rad = math.pi / 180
quatRotn2rad = gyroGain * deg2rad/1000000
rad2deg = 1/deg2rad
offset = 0
prevRollZ = 0
fullRollZ = 0

lastquatRotn = 0
quatRotnSumX = 0
quatRotnSumY = 0
quatRotnSumZ = 0

"""Iterate down the rows"""
for i in range(1,apogee,1):
    
    dt = quatRotn.at[i, 'timeStamp'] - quatRotn.at[i-1, 'timeStamp']
    
    quatRotnSumX += quatRotn.at[i, 'gyroX'] * dt
    quatRotnSumY += quatRotn.at[i, 'gyroY'] * dt
    quatRotnSumZ += quatRotn.at[i, 'gyroZ'] * dt
    
    if quatRotn.at[i, 'timeStamp'] - lastquatRotn > 10000:
        
        lastquatRotn = quatRotn.at[i, 'timeStamp']
        
        dx = (quatRotnSumX + offset) * quatRotn2rad
        dy = (quatRotnSumY + offset) * quatRotn2rad
        dz = (quatRotnSumZ + offset) * quatRotn2rad
        
        """Compute quaternion derivative"""
        quatRotn.at[i, 'QuatDiff1'] = 0.5 * (-1 * dx * quatRotn.at[i-1, 'Quat2'] - dy * quatRotn.at[i-1, 'Quat3'] - dz * quatRotn.at[i-1, 'Quat4'])
        quatRotn.at[i, 'QuatDiff2'] = 0.5 * (     dx * quatRotn.at[i-1, 'Quat1'] - dy * quatRotn.at[i-1, 'Quat4'] + dz * quatRotn.at[i-1, 'Quat3']) 
        quatRotn.at[i, 'QuatDiff3'] = 0.5 * (     dx * quatRotn.at[i-1, 'Quat4'] + dy * quatRotn.at[i-1, 'Quat1'] - dz * quatRotn.at[i-1, 'Quat2']) 
        quatRotn.at[i, 'QuatDiff4'] = 0.5 * (-1 * dx * quatRotn.at[i-1, 'Quat3'] + dy * quatRotn.at[i-1, 'Quat2'] + dz * quatRotn.at[i-1, 'Quat1']) 
        
        """Update the quaternion"""
        quatRotn.at[i, 'Quat1'] = quatRotn.at[i-1, 'Quat1'] + quatRotn.at[i, 'QuatDiff1']
        quatRotn.at[i, 'Quat2'] = quatRotn.at[i-1, 'Quat2'] + quatRotn.at[i, 'QuatDiff2']
        quatRotn.at[i, 'Quat3'] = quatRotn.at[i-1, 'Quat3'] + quatRotn.at[i, 'QuatDiff3']
        quatRotn.at[i, 'Quat4'] = quatRotn.at[i-1, 'Quat4'] + quatRotn.at[i, 'QuatDiff4']
        
        """re-normalize"""
        quatLen = pow(pow(quatRotn.at[i, 'Quat1'],2) + pow(quatRotn.at[i, 'Quat2'],2) + pow(quatRotn.at[i, 'Quat3'],2) + pow(quatRotn.at[i, 'Quat4'],2), -0.5)
        quatRotn.at[i, 'Quat1'] *= quatLen
        quatRotn.at[i, 'Quat2'] *= quatLen
        quatRotn.at[i, 'Quat3'] *= quatLen
        quatRotn.at[i, 'Quat4'] *= quatLen
            
        """compute the components of the rotation matrix"""
        a = quatRotn.at[i, 'Quat1']
        b = quatRotn.at[i, 'Quat2']
        c = quatRotn.at[i, 'Quat3']
        d = quatRotn.at[i, 'Quat4']
        a2 = a*a
        b2 = b*b
        c2 = c*c
        d2 = d*d
        ab = a*b
        ac = a*c
        ad = a*d
        bc = b*c
        bd = b*d
        cd = c*d
        
        """Compute rotation matrix"""
        quatRotn11 = a2 + b2 - c2 - d2
        """quatRotn12 = 2 * (bc - ad)"""
        """quatRotn13 = 2 * (bd + ac)"""
        quatRotn21 = 2 * (bc + ad)
        """quatRotn22 = a2 - b2 + c2 - d2"""
        """quatRotn23 = 2 * (cd - ab)"""
        quatRotn31 = 2 * (bd - ac)
        quatRotn32 = 2 * (cd + ab)
        quatRotn33 = a2 - b2 - c2 + d2
        
        """compute 3D orientation"""
        quatRotn.at[i, 'pitchX'] = math.atan2(quatRotn32, quatRotn33) 
        quatRotn.at[i, 'yawY'] = math.asin(-1*quatRotn31)
        quatRotn.at[i, 'rollZ'] = math.atan2(quatRotn21, quatRotn11)
        tanYaw = math.tan(quatRotn.at[i, 'yawY'])
        tanYaw = tanYaw * tanYaw
        tanPitch = math.tan(quatRotn.at[i, 'pitchX'])
        tanPitch = tanPitch * tanPitch
        quatRotn.at[i, 'offVert'] = math.atan(pow(tanYaw + tanPitch, 0.5))
        """convert to radians"""
        quatRotn.at[i, 'pitchX']  *= rad2deg
        quatRotn.at[i, 'yawY']    *= rad2deg
        quatRotn.at[i, 'rollZ']   *= rad2deg
        quatRotn.at[i, 'offVert'] *= rad2deg
        
        '''reset the averaging variables'''
        quatRotnSumX = 0
        quatRotnSumY = 0
        quatRotnSumZ = 0
        
        prevRollZ = quatRotn.at[i-1, 'rollZ']
        if quatRotn.at[i, 'rollZ'] + fullRollZ *360 - prevRollZ > 180:
          fullRollZ -= 1
        elif quatRotn.at[i, 'rollZ'] + fullRollZ *360 - prevRollZ < -180:
          fullRollZ += 1
        quatRotn.at[i, 'rollZ'] += fullRollZ * 360
 
    else:
        quatRotn.at[i, 'Quat1'] = quatRotn.at[i-1, 'Quat1']
        quatRotn.at[i, 'Quat2'] = quatRotn.at[i-1, 'Quat2']
        quatRotn.at[i, 'Quat3'] = quatRotn.at[i-1, 'Quat3']
        quatRotn.at[i, 'Quat4'] = quatRotn.at[i-1, 'Quat4']
        quatRotn.at[i, 'QuatDiff1'] = quatRotn.at[i-1, 'QuatDiff1']
        quatRotn.at[i, 'QuatDiff2'] = quatRotn.at[i-1, 'QuatDiff2']
        quatRotn.at[i, 'QuatDiff3'] = quatRotn.at[i-1, 'QuatDiff3']
        quatRotn.at[i, 'QuatDiff4'] = quatRotn.at[i-1, 'QuatDiff4']
        quatRotn.at[i, 'pitchX'] = quatRotn.at[i-1, 'pitchX']
        quatRotn.at[i, 'yawY'] = quatRotn.at[i-1, 'yawY']
        quatRotn.at[i, 'rollZ'] = quatRotn.at[i-1, 'rollZ']
        quatRotn.at[i, 'offVert'] = quatRotn.at[i-1, 'offVert']
        
quatRotn.head(apogee).plot(x = 'flightTime', y = ['pitchX', 'yawY', 'offVert'], ylabel = 'Quaternion Rotation (degrees)')
quatRotn.loc[apogee, 'offVert']
rotate = quatRotn['offVert']
rotate.max()


"""-------------------------------
Compare DCM2D to quaternion differentials
-------------------------------"""
compare = pd.merge(
    origRotn,
    quatRotn[['timeStamp','pitchX','yawY','rollZ','offVert']],
    how="inner",
    on='timeStamp',
    left_on=None,
    right_on=None,
    left_index=False,
    right_index=False,
    sort=True,
    suffixes=("_DCM2D", "_Quat"),
    copy=True,
    indicator=False,
    validate=None,
)

rotnCompare = pd.merge(
    compare,
    rotn[['timeStamp','pitchX','yawY','rollZ','offVert']],
    how="inner",
    on='timeStamp',
    left_on=None,
    right_on=None,
    left_index=False,
    right_index=False,
    sort=True,
    copy=True,
    indicator=False,
    validate=None,
)
rotnCompare['offVert']*=0.1
rotnCompare['VertQuat-Onboard'] = rotnCompare['offVert_Quat'] - rotnCompare['offVert']
rotnCompare['VertQuat-DCM2D'] = rotnCompare['offVert_Quat'] - rotnCompare['offVert_DCM2D']
rotnCompare['RollQuat-Onboard'] = rotnCompare['rollZ_Quat'] - rotnCompare['rollZ']
rotnCompare['RollQuat-DCM2D'] = rotnCompare['rollZ_Quat'] - rotnCompare['rollZ_DCM2D']
rotnCompare.head(apogee).plot(
    x = 'flightTime', 
    y = ['offVert', 'offVert_Quat', 'offVert_DCM2D'], 
    xlabel = "Flight Time", 
    ylabel="Degrees Off-Vertical", 
    label = ["Onboard","Quaternion","DCM2D"],
    title="Off-Vertical Rotation Method Comparison")
rotnCompare.head(apogee).plot(
    x = 'flightTime',
    y = ['VertQuat-Onboard','VertQuat-DCM2D'],
    xlabel = "Flight Time",
    ylabel = "Error (Degrees)",
    label = ["Quat-Onboard","Quat-DCM2D"],
    title = "Off Vertical Error from 64-bit Quaternion")
rotnCompare.head(apogee).plot(
    x = 'flightTime',
    y = ['rollZ', 'rollZ_Quat','rollZ_DCM2D'],
    label = ["Onboard","Quaternion","DCM2D"],
    title = "Roll Method Comparison")
rotnCompare.head(apogee).plot(
    x = 'flightTime',
    y = ['RollQuat-Onboard','RollQuat-DCM2D'],
    xlabel = "Flight Time",
    ylabel = "Error (Degrees)",
    label = ["Quat-Onboard","Quat-DCM2D"],
    title = "Roll Error from 64-bit Quaternion")
#compare.head(apogee).loc[:,['offVert_Quat', 'offVert_DCM2D']].max()

"""-------------------------------
3D PHYSICS MODEL
-------------------------------"""
physMod = pd.merge(
    df['timeStamp','accelX','accelY','accelZ'],
    quatRotn[['timeStamp','pitchX','yawY','rollZ','offVert']],
    how="inner",
    on='timeStamp',
    left_on=None,
    right_on=None,
    left_index=False,
    right_index=False,
    sort=True,
    #suffixes=("_DCM2D", "_Quat"),
    copy=True,
    indicator=False,
    validate=None,
)




"""-------------------------------
BATTERY VOLTAGE PLOT
-------------------------------"""
df['battVolt'] = df['battVolt'].fillna(method = 'pad')
df.plot(
        x = 'flightTime',
        y = 'battVolt',
        xlabel = 'Flight Time',
        ylabel = 'Battery Voltage',
        legend = False,
        title = 'Battery Voltage')

"""-------------------------------
BAROMETRIC TEMPERATURE PLOT
-------------------------------"""
baro = df.loc[:, ['flightTime','baroPress','baroAlt','altMoveAvg', 'baroVel','baroTemp']]
baro = baro[baro['baroTemp'].notnull()].reset_index(drop = True)
baro.plot(
    x = 'flightTime',
    y = 'baroTemp',
    xlabel = 'Flight Time',
    ylabel = 'Temperature (C)',
    title = 'Barometric Temperature')

"""-------------------------------
MAGNETOMETER DATA PLOT
-------------------------------"""
#import statistics as stats
mag = df.loc[:, ['timeStamp','magX','magY','magZ', 'fltEvents']]
mag = mag[mag['magX'].notnull()].reset_index(drop = True)
'''rotn = df.loc[::15, ['timeStamp','gyroX','gyroY','gyroZ']].reset_index(drop = True)'''
mag['flightTime'] = mag['timeStamp']/1000000
magTest = mag['fltEvents'].index[0]
magApogee = mag[mag['fltEvents'].str[4] == "1"].index[0]
if magApogee >= mag.index.max()-10: magApogee = mag[mag['fltEvents'].str[6] == "1"].index[0]

#magXoffset = stats.mean(mag['magX'].head(200))
#magYoffset = stats.mean(mag['magY'].head(200))
magXoffset = min(mag['magX'].head(200)) + (max(mag['magX'].head(200)) - min(mag['magX'].head(200)))/2
magYoffset = min(mag['magY'].head(200)) + (max(mag['magY'].head(200)) - min(mag['magY'].head(200)))/2
magZoffset = mag.at[1,'magZ']

"""magNorm = 1/math.sqrt(magX0**2 + magY0**2 + magZ0**2)
magX0 *= magNorm
magY0 *= magNorm
magZ0 *= magNorm"""

mag['magX_corrected']=mag['magX'] - magXoffset
mag['magY_corrected']=mag['magY'] - magYoffset
mag['magZ_corrected']=mag['magZ'] - magZoffset

magX0 = mag.at[1, 'magX_corrected']
magY0 = mag.at[1, 'magY_corrected']
magZ0 = mag.at[1, 'magZ_corrected']

mag.head(200).plot(
        x = 'magX_corrected',
        y = 'magY_corrected',
        xlabel = 'mag X',
        ylabel = 'mag Y',
        title = 'Corrected X-Y Magnetometer Output')

mag.head(magApogee).plot(
    x = 'timeStamp',
    y = ['magX_corrected','magY_corrected','magZ_corrected'])

"""-------------------------------
MAGNETOMETER ROTATION PLOT
-------------------------------"""
valVect1 = magX0**2 + magY0**2
lenVect1 = math.sqrt(valVect1)
valVect2 = 0
mag.at[0, 'magRoll'] = 0
mag.at[0, 'magPitch'] = 0
mag.at[0, 'magYaw'] = 0
mag.at[0, 'magOffVert'] = 0
mag.at[0, 'cosMagRoll'] = 1

for i in range(1, magApogee, 1):
    
    dotProd = magX0 * mag.at[i,'magX_corrected'] + magY0 * mag.at[i, 'magY_corrected']
    valVect2 = mag.at[i, 'magX_corrected']**2 + mag.at[i, 'magY_corrected']**2
    lenVect2 = math.sqrt(valVect2)

    """compute roll"""
    cosMagRoll = dotProd / (lenVect1 * lenVect2)  
    mag.at[i, 'magRoll'] = math.acos(cosMagRoll) * 180 / math.pi

    """compute off-vertical"""
    dotProd = valVect2;
    valVect3 = valVect2 + mag.at[i, 'magZ_corrected']**2
    lenVect3 = math.sqrt(valVect3)
    cosOffVert = dotProd / (lenVect2 * lenVect3)
    mag.at[i, 'magOffVert'] = math.acos(cosOffVert)
    
    """compute pitch and yaw"""
    mag.at[i, 'magPitch'] = mag.at[i, 'magOffVert'] * cosMagRoll * 180/math.pi
    mag.at[i, 'magYaw'] = mag.at[i, 'magOffVert'] * math.sqrt(1-cosMagRoll**2) * 180/math.pi
    mag.at[i, 'magOffVert'] *= 180/math.pi
    mag.at[i, 'cosMagRoll'] = cosMagRoll

mag.head(magApogee).plot(
    x = 'timeStamp',
    y = ['magPitch','magYaw','magOffVert'],
    title = 'Magnetometer Calculated Off-Vertical')

mag.head(magApogee).plot(
    x = 'flightTime',
    y = 'magRoll',
    title = 'Magnetometer Calculated Roll')

"""-------------------------------
MAGNETOMETER COMPARISON TO QUATERNION
-------------------------------"""
magCompare = pd.merge(
    mag,
    rotn[['timeStamp','pitchX','yawY','rollZ','offVert']],
    how="inner",
    on='timeStamp',
    left_on=None,
    right_on=None,
    left_index=False,
    right_index=False,
    sort=True,
    copy=True,
    indicator=False,
    validate=None,
)
magCompare['offVert'] *= .1
magCompare['pitchX'] *= .1
magCompare['yawY'] *= .1
for i in range(0, magCompare.index.max(),1):
    magCompare.at[i, 'cosRoll'] = math.cos(magCompare.at[i, 'rollZ'] * math.pi/180)

magCompare2 = pd.merge(
    magCompare,
    origRotn[['timeStamp','rollZ']],
    how="inner",
    on='timeStamp',
    left_on=None,
    right_on=None,
    left_index=False,
    right_index=False,
    sort=True,
    suffixes=("_Quat","_DCM2D"),
    copy=True,
    indicator=False,
    validate=None,
)

for i in range(0, magCompare.index.max(),1):
    magCompare2.at[i, 'cosDCM2DRoll'] = math.cos(magCompare2.at[i, 'rollZ_DCM2D'] * math.pi/180)

magCompare.head(magApogee).plot(
    x = 'flightTime',
    y = ['magOffVert','offVert'],
    xlabel = "Flight Time", 
    ylabel="Degrees Off-Vertical", 
    label = ["Magnetometer","Quaternion"],
    title="Off-Vertical Rotation Method Comparison")

magCompare2.head(magApogee).plot(
    x = 'flightTime',
    y = ['cosMagRoll','cosRoll','cosDCM2DRoll'],
    xlabel = "Flight Time", 
    ylabel="Cosine of Roll", 
    label = ["Magnetometer","Quaternion","DCM2D"],
    title="Roll Comparison")

for i in range(0, magApogee, 1):
    magCompare2.at[i, 'cosDiff'] = abs(magCompare2.at[i,'cosDCM2DRoll'] - magCompare2.at[i, 'cosMagRoll'])

magCompare2['cosDiff'].max()