import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
from environment.computeWalkableSurfaceConnectivity import *
from environment.walkableSurfacesPlotter import *
from pathplanner.footstepPathHomotopyOptimizer import *
from pathplanner.footstepPathPlotter import *

env = os.environ["MPP_PATH"]+"mpp-environment/urdf/"

#####################################################
fname = "quatro_homotopy.urdf"
xstart = np.array([0,0,2])
xgoal = np.array([0,8,2])
#####################################################

fname = env+fname

stime = np.array((0.0,0.0))
shtime = np.array([0.0])
totaltime = 0.0

Nruns = 10
for i in range(0,Nruns):
        time = np.array((0,0))
        htime = np.array([0])
        time = computeWalkableSurfaceConnectivity(fname,clipper=False)
        htime = footstepPathHomotopyOptimizer(xstart,xgoal,DEBUG=False)
        R = len(htime)
        stime[0] += time[0]/float(Nruns)
        stime[1] += time[1]/float(Nruns)
        totaltime += (np.sum(time)+np.sum(htime))/float(Nruns)
        shtime += np.sum(htime)/float(Nruns)

print "==============================================="
print " TIMER (AVERAGED OVER",Nruns,"RUNS)"
print "==============================================="
print "R      homotopies:",R
print "T_W(s) extract walkable surfaces:",stime[0]
print "T_G(s) connectivity walkable surfaces:",stime[1]
print "T_P(s) homotopy planning:",shtime
print "T(s)   total sum:",totaltime
print "==============================================="
