# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 19:56:58 2021

@author: Sparky
"""

"""-------------------------------
IMPORT DATA FILE
-------------------------------"""
import win32gui, win32con, os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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
    #dtype={'fltEvents': str}, 
    engine='python', 
    delimiter = ",", 
    skiprows = 2, 
    skipfooter= 2, 
    index_col = False
    )

apogee = df[df['event']>= 5].index[0]
burnout = df[df['event']== 2].index[0]

df.dtypes
"""-------------------------------
ACCELERATION PLOT
-------------------------------"""
df['acceleration']=df['acceleration'].astype('float') 
if burnout > 0:
    df.head(burnout).plot(
        x = 'time',
        xlabel = 'Flight Time',
        y = 'acceleration',
        ylabel = 'Accel (m/s2)',
        title = 'Booster Motor',
        legend = False)
    
# a = df.loc[burnout+45:burnout+80,['time','acceleration']]
# a.loc[87:101,'acceleration']*=-1
# a.plot(
#        x = 'time',
#        y = 'acceleration')

"""-------------------------------
VELOCITY PLOT
-------------------------------"""
df.head(apogee).plot(
    x = 'time',
    xlabel = 'Flight Time',
    y = 'velocity',
    ylabel = 'Velocity (m/s)',
    title = 'Velocity to Apogee',
    legend = False)

df.plot(
    x = 'time',
    xlabel = 'Flight Time',
    y = 'velocity',
    ylabel = 'Velocity (m/s)',
    title = 'Velocity to Touchdown',
    legend = False)

"""-------------------------------
ALTITUDE PLOT
-------------------------------"""
packetRate = (df.loc[:,'packetNum'].index.max()/4)/df.loc[:,'packetNum'].max()
df.head(apogee).plot(
    x = 'time',
    xlabel = 'Flight Time',
    y = ['altitude','gpsAlt'],
    ylabel = 'Altitude (m)',
    title = 'Altitude to Apogee',
    label = ['Sensor Fusion Alt','SatNav Alt'])

plot1 = df.plot(
    x = 'time',
    xlabel = 'Flight Time',
    y = ['altitude','gpsAlt'],
    ylabel = 'Altitude (m)',
    title = 'Altitude to Touchdown, DataRate: ' + str(packetRate*100)[0:4] +'%',
    label = ['Sensor Fusion Alt','SatNav Alt'])

df.plot(
    x = 'time',
    xlabel = 'Flight Time',
    y = 'signalStrength',
    label = 'Signal Strength',
    secondary_y = True,
    ax = plot1).set_ylabel('Signal Srength (dbm)')

"""-------------------------------
ROTATION PLOT
-------------------------------"""
plot1 = df.head(apogee).plot(
    x = 'time',
    xlabel = 'Flight Time',
    y = 'offVert',
    ylabel = 'Degrees Off Vertical',
    title = 'Rotation to Apogee',
    label = 'Off Vertical')

df.head(apogee).plot(
    x = 'time',
    y = 'spin',
    secondary_y = True,
    label = 'Spin',
    ax = plot1).set_ylabel('Degrees of Spin')

"""-------------------------------
GNSS PLOT
-------------------------------"""
# df.loc[:, 'latitude']=df.loc[:,'gpsLat'].str[1:10].astype('float32')
# df.loc[:, 'longitude']=df.loc[:,'gpsLon'].str[1:10].astype('float32')
# df['latitude'] -= df.at[0, 'latitude']
# df['longitude'] -= df.at[0, 'longitude']
# df['latitude'] *= 1.15*5280
# df['longitude'] *= 0.91*5280
# df.plot(
#     x = 'latitude',
#     y = 'longitude',
#     title = 'Ground Track from Pad (ft)',
#     legend = False,
#     ylabel = 'South-North',
#     xlabel = 'West-East')

# altRange = (df['gpsAlt'].max() - df['gpsAlt'].min())*3.2808

# gph = plt.figure(figsize=(15,8)).gca(projection='3d')
# gph.plot(df['latitude'], df['longitude'], df['gpsAlt']*3.2808, "b.--")
# gph.set_xlim(-1*altRange/2, altRange/2)
# gph.set_ylim(-1*altRange/2, altRange/2)
# gph.set_zlim(df['gpsAlt'].min()*3.2808, df['gpsAlt'].max()*3.2808)
# gph.set_title('Flight Progression', fontsize= 20)
# gph.set_xlabel('West-East', fontsize = 14)
# gph.set_ylabel('South-North', fontsize = 14)
# gph.set_zlabel('Altitude (ft)', fontsize = 14)

"""-------------------------------
PACKET PLOT
-------------------------------"""
# maxPackets = df.loc[:,'packetNum'].max()
# data = np.linspace(1,maxPackets, maxPackets, dtype = int)
# pkts = pd.dataframe(data,'pkt')
# pkts[0,'recieved'] = 1
# for j in range(maxPackets):
#     if pkts['']  
