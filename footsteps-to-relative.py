import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
import pickle
from math import acos
import numpy as np
from scipy.interpolate import interp1d
from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

output_folder = os.environ["MPP_PATH"]+"mpp-environment/output/"
svFootFname = output_folder+"/xpathFoot.dat"
fPath = pickle.load( open( svFootFname, "rb" ) )

fig=figure(1)
ax = fig.gca()

#for i in range(0,len(fPath)):
#        X = [fPath[i,0],fPath[i,0]+0.1*fPath[i,3]]
#        Y = [fPath[i,1],fPath[i,1]+0.1*fPath[i,4]]
#        ax.plot(X,Y)

f = interp1d(fPath[:,0],fPath[:,1], kind='linear')
xl = min(fPath[:,0])
xu = max(fPath[:,0])

xnew = np.linspace(min(fPath[:,0]), max(fPath[:,0]), 1000)
ax.plot(xnew,f(xnew))
an = fPath[0,3:6]
x = np.array((1,0,0))
tstart= acos(np.dot(an,x))
foot = np.array((xnew[0],f(xnew[0]),0.0,tstart))

#foot = np.vstack([foot,np.array((0.0,-0.1,0.0,0.0))])
foot = np.array((0.0,-0.1,0.0,0.0))
#print foot
plot(xnew[0],f(xnew[0]),'ok')

ctr=0
while ctr<len(xnew)-1:
        d=0.0
        ctrstart = ctr
        ##absolute position of left foot
        #an = fPath[ctr,3:6]
        #tctr= acos(np.dot(an,x))
        tctr=0
        leftFoot = np.array((xnew[ctr],f(xnew[ctr]),tctr))

        while d<0.3 and ctr<len(xnew)-1:
                vc = np.array((xnew[ctr],f(xnew[ctr])))
                vn = np.array((xnew[ctr+1],f(xnew[ctr+1])))
                d+=np.linalg.norm(vc-vn)
                ctr=ctr+1

        #an = fPath[ctr,3:6]
        #tctr= acos(np.dot(an,x))
        tctr=0
        rightFoot = np.array((xnew[ctr],f(xnew[ctr]),tctr))

        d=0.0
        ctr=ctrstart
        while d<0.1 and ctr<len(xnew)-1:
                vc = np.array((xnew[ctr],f(xnew[ctr])))
                vn = np.array((xnew[ctr+1],f(xnew[ctr+1])))
                d+=np.linalg.norm(vc-vn)
                ctr=ctr+1

        #an = fPath[ctr,3:6]
        #tctr= acos(np.dot(an,x))
        tctr=0
        leftFoot = np.array((xnew[ctr],f(xnew[ctr]),tctr))
        plot(rightFoot[0],rightFoot[1],'or')
        plot(leftFoot[0],leftFoot[1],'ob')
        #plot(xnew[ctr],f(xnew[ctr]),'ob')
        #print d,ctr

plt.show()
