import matplotlib.pyplot as plt
import numpy as np
import csv
# Read Mixer Input Data
mixer_table = [[ -1.000000,  0.5,  -0.5, 1.000000, 1.000000 ],
[ -1.000000,  -0.5,  0.5, 1.000000, 1.000000 ],
[ 1.000000, 0.5,   0.5, 1.000000, 1.000000 ],
[ 1.000000,-0.5, -0.5, 1.000000, 1.000000 ]]

with open("mixer_input.txt", 'rt') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    roll = []
    pitch = []
    yaw = []
    thrust = []
    mout1 = []
    mout2 = []
    mout3 = []
    mout4 = []
    mmax = []
    mmin = []
    upperbound_gap = []
    lowerbound_gap = []
    for row in readCSV:
        _roll = row[0]
        _pitch = row[1]
        _yaw = row[2]
        _thrust = row[3]
        _mout1 = float(_roll) * mixer_table[0][0] + float(_pitch) * mixer_table[0][1] + float(_yaw) * mixer_table[0][2] + float(_thrust) * mixer_table[0][3]
        _mout2 = float(_roll) * mixer_table[1][0] + float(_pitch) * mixer_table[1][1] + float(_yaw) * mixer_table[1][2] + float(_thrust) * mixer_table[1][3]
        _mout3 = float(_roll) * mixer_table[2][0] + float(_pitch) * mixer_table[2][1] + float(_yaw) * mixer_table[2][2] + float(_thrust) * mixer_table[2][3]
        _mout4 = float(_roll) * mixer_table[3][0] + float(_pitch) * mixer_table[3][1] + float(_yaw) * mixer_table[3][2] + float(_thrust) * mixer_table[3][3]
        _mmax = max([_mout1, _mout2, _mout3,_mout4])
        _mmin = min([_mout1, _mout2, _mout3,_mout4])
        mout1.append((_mout1 + 1 ) /2)
        mout2.append((_mout2 + 1 ) /2)
        mout3.append((_mout3 + 1 ) /2)
        mout4.append((_mout4 + 1 ) /2)
        mmin.append(_mmin)
        mmax.append(_mmax)
        _uppgap = 1 - _mmax
        if _uppgap < 0:
            _uppgap = 0
        _lowgap = _mmin + 1
        if _lowgap < 0:
            _lowgap = 0
        upperbound_gap.append(_uppgap)
        lowerbound_gap.append(_lowgap)
        roll.append(_roll)
        pitch.append(_pitch)
        yaw.append(_yaw)
        thrust.append(_thrust)

# Read Mixer Output Data
with open("mixer_output.txt", 'rt') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    mixed1 = []
    mixed2 = []
    mixed3 = []
    mixed4 = []
    for row in readCSV:
        _mixed1 = row[0]
        _mixed2 = row[1]
        _mixed3 = row[2]
        _mixed4 = row[3]
        mixed1.append(_mixed1)
        mixed2.append(_mixed2)
        mixed3.append(_mixed3)
        mixed4.append(_mixed4)

const1 = np.ones_like(mixed1)
const0 = np.zeros_like(mixed1)

ax1 = plt.subplot2grid((4,2), (0,0))
ax1repeat = plt.subplot2grid((4,2), (0,1))
ax2 = plt.subplot2grid((4,2), (1,1))
ax3 = plt.subplot2grid((4,2), (2,0))
ax4 = plt.subplot2grid((4,2), (1,0))
ax5 = plt.subplot2grid((4,2), (2,1))
ax6 = plt.subplot2grid((4,2), (3,0))
ax7 = plt.subplot2grid((4,2), (3,1))
# f, (ax1, ax2, ) = plt.subplots(2, 1, sharex=True, figsize=(20, 10))
samples = range(len(roll))
ax1.plot(samples, roll, 'r', label ='Roll', linewidth = 2)
ax1.plot(samples, pitch, 'k', label = 'Pitch', linewidth = 2)
ax1.plot(samples, yaw, 'b', label = 'Yaw', linewidth = 2)
ax1.plot(samples, thrust, 'r--', label = 'Thrust', linewidth = 2)
ax1repeat.plot(samples, roll, 'r', label ='Roll', linewidth = 2)
ax1repeat.plot(samples, pitch, 'k', label = 'Pitch', linewidth = 2)
ax1repeat.plot(samples, yaw, 'b', label = 'Yaw', linewidth = 2)
ax1repeat.plot(samples, thrust, 'r--', label = 'Thrust', linewidth = 2)
ax1.legend()
samples = range(len(mixed1))
ax2.plot(samples, mixed1, '--', label = 'M1', linewidth = 2)
ax2.plot(samples, mout1, label='unscaled')
ax2.axhline(y=1, color='r', linestyle='-')
ax2.axhline(y=0, color='r', linestyle='-')
ax3.plot(samples, mixed2, '--', label = 'M2', linewidth = 2)
ax3.plot(samples, mout2, label='unscaled')
ax3.axhline(y=1, color='r', linestyle='-')
ax3.axhline(y=0, color='r', linestyle='-')
ax4.plot(samples, mixed3, '--', label = 'M3', linewidth = 2)
ax4.plot(samples, mout3, label='unscaled')
ax4.axhline(y=1, color='r', linestyle='-')
ax4.axhline(y=0, color='r', linestyle='-')
ax5.plot(samples, mixed4, '--', label = 'M4', linewidth = 2)
ax5.plot(samples, mout4, label='unscaled')
ax5.axhline(y=1, color='r', linestyle='-')
ax5.axhline(y=0, color='r', linestyle='-')
ax6.plot(samples, mmax, label = 'max out')
ax6.plot(samples, mmin, label = 'min out')
ax6.axhline(y=-1, color='r', linestyle='-')
ax6.axhline(y=1, color='r', linestyle='-')
ax7.plot(samples, upperbound_gap, label='Upper Bound Gap')
ax7.plot(samples, lowerbound_gap, label='Lower Bound Gap')
ax7.axhline(y=1, color='r', linestyle='-')

ax2.legend()
ax3.legend()
ax4.legend()
ax5.legend()
ax6.legend()
ax7.legend()
ax1.set_ylim([-1.2,1.2])
ax2.set_ylim([-2.5,2.5])
ax3.set_ylim([-2.5,2.5])
ax4.set_ylim([-2.5,2.5])
ax5.set_ylim([-2.5,2.5])
plt.show()


