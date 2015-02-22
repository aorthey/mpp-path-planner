import numpy as np
from pylab import *
import os
import matplotlib 
import matplotlib.pyplot as plt

x = np.arange(0,10)
##in minutes
rrt = np.array((217,367,235,574,397,44,979,165,2637,123))
pp = 35*np.ones((10,1))


params = {'legend.fontsize': 20,
          'legend.linewidth': 2}
plt.rcParams.update(params)
rcParams['legend.loc'] = 'upper left'

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}
axfont = {'family' : 'normal',
        'weight' : 'normal',
        'size'   : 16}
fig = plt.figure()
ax = fig.gca()

lrrt = ax.semilogy(x,rrt,'-or',linewidth=2,markersize=10,label='RRT')
lpp = ax.semilogy(x,pp,'-ob',linewidth=2,markersize=10,label='PP')

ax.set_ylim((0, 3000))
ax.legend()


ax.set_axis_bgcolor('white')
ax.patch.set_facecolor('white')
plt.xlabel("Instances", **axfont)
plt.ylabel("Time(s)", **axfont)


matplotlib.rc('font', **font)
savefig('results-trim.png', bbox_inches='tight')
os.system('cp results-trim.png /home/aorthey/git/15-orthey-iros/images/simulations/')
plt.show()
