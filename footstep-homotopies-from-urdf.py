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
#fname = "wall_simplified.urdf"
#fname = "wall-extension.urdf"
fname = "koroibot_stair_cases.sdf"
#fname = "staircase_stones.urdf"
#fname = "quatro_homotopy.urdf"
#xstart = np.array([0,0,2])
#xgoal = np.array([0,8,2])
#####################################################
if fname == "staircase_stones.urdf" or\
   fname == "quatro_homotopy.urdf":
        xstart = np.array([0,0,2])
        xgoal = np.array([0,8,2])
elif fname=="koroibot_stair_cases.sdf":
        xstart = np.array([0.1,0.6,0.1])
        xgoal = np.array([1.5,-1.4,0.1])
else:
        xstart=np.array((-0.5,2.0,0.05))
        xgoal=np.array((0.5,-2.0,0.05))
#####################################################

fname = env+fname

time = np.array((0,0))
htime = np.array([0])
#time = computeWalkableSurfaceConnectivity(fname)
walkableSurfacesPlotter(fname,plotscene=True)

htime = footstepPathHomotopyOptimizer(xstart,xgoal)

K = len(htime)
footstepPathPlotter(fname, K, plotscene=False)

print "==============================================="
print " TIMER "
print "==============================================="
print "extract walkable surfaces:",time[0]
print "connectivity walkable surfaces:",time[1]
print "K homotopies:",len(htime)
print "Homotopy planning:",np.sum(htime)
print "total sum:",np.sum(time)+np.sum(htime)
print "==============================================="

