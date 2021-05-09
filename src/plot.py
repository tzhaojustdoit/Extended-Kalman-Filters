import numpy as np
import matplotlib.pyplot as plt
N = 4
Lidar = [0.122191,
         0.0983799,
         0.582513,
         0.456699]
Radar = [0.19172,
         0.279417,
         0.556905,
         0.655558]
Both = [0.0972256,
        0.0853761,
        0.450855,
        0.439588]
ind = np.arange(N)  # the x locations for the groups
width = 0.35
fig = plt.figure()
ax = fig.add_axes([0, 0, 1, 1])
ax.bar(ind + 0.00, Lidar, color='b', width=0.25)
ax.bar(ind + 0.25, Radar, color='g', width=0.25)
ax.bar(ind + 0.50, Both, color='r', width=0.25)
ax.legend(labels=['Lidar Only', 'Radar Only', 'Both'])
ax.set_ylabel('RMSE')

plt.show()
