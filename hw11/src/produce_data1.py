#! /usr/bin/env python

import numpy as np
import pylab as plt
import csv

time_stamp = []
num_list = []
receve_list= [-41.3476,-10.56936,-0.09996,6.33969,45.89824,45.62117,44.61856,51.14404,60.73781,86.47371,131.77911,118.70147,198.3733,243.21885,223.93135,261.76944,295.08487,304.72116,347.15858,390.1038,445.32955,502.8035,544.69698,553.10905,619.93109,679.36774,691.80668,775.08624,819.29884,877.79287,979.42744,1056.82776,1073.63762,1145.40761,1212.50599,1308.97609,1343.82864,1447.64044,1513.31818,1633.95831,1702.98397,1746.72186,1868.41031,1910.65399,2030.43845,2135.39993]
kf_list = []

for t in range(0,46,1):
    d = (2.0 * t) + ( t**2.0 )
    num_list.append(d)
    time_stamp.append(t)

print(num_list)
print(receve_list)

with open('/home/ncrl/robot_ws/src/Homework/hw11/src/output.csv', 'w') as csvfile:
  writer = csv.writer(csvfile)
  writer.writerow(num_list)
  writer.writerow(receve_list)

with open('/home/ncrl/robot_ws/src/Homework/hw11/src/data.csv', 'r') as csvfile:
  reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
  for row in reader:
    kf_list = row
  kf_list = filter(None, kf_list)
  print(kf_list)

plt.figure(figsize=(6,8))
plt.subplot(311)
plt.title('The Original Signal')
plt.plot(time_stamp[0:46],num_list[0:46])

plt.subplot(312)
plt.title('Noise Signal')
plt.plot(time_stamp[0:46],receve_list[0:46])

plt.subplot(313)
plt.title('After KF Signal')
plt.plot(time_stamp[0:46],kf_list[0:46])

plt.show()
plt.tight_layout()
