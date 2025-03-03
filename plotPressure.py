# Read pressure data vs time and plot
# 3/2/2025 J.Beale

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
import datetime
import pandas as pd
import os, sys

# ================================================
dir = r"C:\Users\beale\Documents\Tiltmeter"
fname1 = r"adc1256-log.csv"
fname2 = r"adc1256-log2.csv"
path1 = os.path.join(dir, fname1)
path2 = os.path.join(dir, fname2)

df1 = pd.read_csv(path1, comment="#")
df1.columns = df1.columns.str.strip()  # get rid of any leading or trailing whitespace in column headers

df2 = pd.read_csv(path2, comment="#")
df2.columns = df2.columns.str.strip()  # get rid of any leading or trailing whitespace in column headers

epoch1 = df1['epoch_time'].values  # seconds after Unix epoch, as np array
values1 = df1['Vavg'].values  # time-averaged ADC reading in microvolts

epoch2 = df2['epoch_time'].values  # seconds after Unix epoch, as np array
values2 = df2['Vavg'].values  # time-averaged ADC reading in microvolts

values2p = np.interp(epoch1, epoch2, values2)

dates1 = [datetime.datetime.fromtimestamp(ts) for ts in epoch1]
dates2 = [datetime.datetime.fromtimestamp(ts) for ts in epoch2]

fig, ax = plt.subplots()
ax.plot(dates1, values1, label=fname1)
#ax.plot(dates2, values2)
ax.plot(dates1, values2p, label=fname2)

diff = values1 - (1.6 * values2p)
ax.plot(dates1, diff, label = "difference")

locator = mdates.AutoDateLocator()
formatter = mdates.ConciseDateFormatter(locator)
ax.xaxis.set_major_locator(locator)
ax.xaxis.set_major_formatter(formatter)

ax.set_xlabel("time (PST)")
ax.set_ylabel("ADC (uV)")
ax.set_title("Pressure vs Time")
ax.grid('both')
ax.legend()

plt.show()
