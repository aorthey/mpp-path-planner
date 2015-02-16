import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
from environment.urdfToWalkableSurfaceConnectivity import *
from environment.walkableSurfacesPlotter import *
from pathplanner.footstepPathHomotopyOptimizer import *
from pathplanner.footstepPathPlotter import *

env = os.environ["MPP_PATH"]+"mpp-environment/urdf/"

#####################################################
#####################################################
#fname = "wall_simplified.urdf"
#fname = "wall.urdf"
#fname = "staircase_stones.urdf"
fname = "quatro_homotopy.urdf"
#####################################################
#####################################################

fname = env+fname


time = computeWalkableSurfaceConnectivity(fname)
#walkableSurfacesPlotter(fname)

xstart = np.array([0,0,2])
xgoal = np.array([0,8,2])
htime = footstepPathHomotopyOptimizer(xstart,xgoal)


print "complete time: ",time+np.sum(htime)
footstepPathPlotter(fname)

