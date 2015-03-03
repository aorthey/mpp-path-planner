import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
from environment.computeWalkableSurfaceConnectivity import *
from environment.walkableSurfacesPlotter import *
from pathplanner.footstepPathHomotopyOptimizer import *
from pathplanner.footstepPathPlotter import *

env = os.environ["MPP_PATH"]+"mpp-environment/urdf/"

#####################################################
#####################################################
fname = "quatro_homotopy.urdf"
xstart = np.array([0,0,2])
xgoal = np.array([0,8,2])
#####################################################

fname = env+fname

time = np.array((0,0))
htime = np.array([0])

time = computeWalkableSurfaceConnectivity(fname,clipper=False)
htime = footstepPathHomotopyOptimizer(xstart,xgoal)

R = len(htime)

print "==============================================="
print " TIMER "
print "==============================================="
print "R      homotopies:",R
print "T_W(s) extract walkable surfaces:",time[0]
print "T_G(s) connectivity walkable surfaces:",time[1]
print "T_P(s) homotopy planning:",np.sum(htime)
print "T(s)   total sum:",np.sum(time)+np.sum(htime)
print "==============================================="

footstepPathPlotter(fname, R, plotscene=True)
