import numpy as np
import csv
import math

samples = 2000
ctrl_input = []
f = open("mixer_input.txt", 'wt')
writer = csv.writer(f)
roll_enable = 1
pitch_enable = 1
yaw_enable = 1
thrust_enable = 1
for sample in range(samples):
    roll = roll_enable * math.sin((sample + 30) * ( 4 * math.pi / samples))
    pitch = pitch_enable * math.sin((sample + 60) * ( 2 * math.pi / samples))
    yaw = yaw_enable * math.sin((sample + 90) * ( 6 * math.pi / samples))
    thrust = thrust_enable * math.sin((sample + 120) * ( 8 * math.pi / samples))
    # ctrl_input.append([[roll, pitch, yaw, thrust]])
    writer.writerow((roll, pitch, yaw, thrust))


