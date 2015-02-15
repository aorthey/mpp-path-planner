import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from timeit import default_timer as timer
import numpy as np
import cvxpy as cvx
import pickle
import networkx as nx
from mathtools.plotter import Plotter,rotFromRPY
from numpy import inf,array,zeros
from cvxpy import *
from math import tan,pi
from mathtools.util import *
from mathtools.linalg import *
from mathtools.timer import *
from mathtools.walkable import WalkableSurface, WalkableSurfacesFromPolytopes
from robot.htoq import *
from robot.robotspecifications import * 
from pathplanner.connectorsGetMiddlePath import * 
from pathplanner.connectorComputeNormal import * 
from pathplanner.surfaceMiddlePath import * 

env_folder = os.environ["MPP_PATH"]+"mpp-environment/output"

Wsurfaces = pickle.load( open( env_folder+"/wsurfaces.dat", "rb" ) )
G_S = pickle.load( open( env_folder+"/graph.dat", "rb" ) )

xstart = np.array([0,0,2])
xgoal = np.array([0,8,2])

[startWS,goalWS]= getNearestWalkableSurfacesFromStartGoalPoint(xstart, xgoal, Wsurfaces)

N=len(Wsurfaces)

for i in range(0,N):
        print "WS",i,"has neighbors:",G_S.neighbors(i)

###############################################################################
# get paths between start and goal surfaces
###############################################################################

paths=list(nx.all_simple_paths(G_S, source=startWS, target=goalWS))

indices = []
print "================================================================"
print "found",len(paths),"homotopy classes in walkable surfaces:"

for i in range(0,len(paths)):
        print paths[i]
        for k in range(0,len(paths[i])):
                indices.append(paths[i][k])

indices = sorted(set(indices))

print "================================================================"
print "relevant walkable surfaces:", indices
print "================================================================"

###############################################################################
# Check which path has the best chances of finding a feasible trajectory
###############################################################################
L = len(paths)

from pathplanner.cvxConstraintFactory import CVXConstraintFactory
from pathplanner.cvxObjectiveFactory import CVXObjectiveFactory
from mathtools.functional_basis import Fpoly

for i in range(0,L):

        P=paths[i]

        N_walkablesurfaces = len(P)
        Wpath = []
        for j in range(0,len(P)):
                Wpath.append(Wsurfaces[P[j]])

        Cfactory = CVXConstraintFactory(N_walkablesurfaces)
        x_WS = Cfactory.getFootpointsFromWalkableSurfaces(Wpath)
        [F,W] = Cfactory.getFunctionalSpace(x_WS)

        Cfactory.addFootpointInFunctionalSpace(x_WS, F, W)
        Cfactory.addDistanceBetweenFootpoints(x_WS)
        Cfactory.addStartPosFootpoint(x_WS,xstart,Wpath)
        Cfactory.addGoalPosFootpoint(x_WS,xgoal,Wpath)
        Cfactory.addFootpointOnSurfaceConstraint(x_WS, Wpath)

        constraints = Cfactory.getConstraints()
        print Cfactory

        Ofactory = CVXObjectiveFactory()
        #Ofactory.addInterpointMinimization(x_WS)
        #Ofactory.addSmoothnessMinimization(W)
        objfunc = Ofactory.getObjectiveFunction()
        objective = Minimize(objfunc)
        print Ofactory

        prob = Problem(objective, constraints)
        timer = Timer("minimizing homotopy "+str(i)+"/"+str(L))
        d = prob.solve(solver=SCS, verbose=True, max_iters=1000)
        print "final cost:",d
        timer.stop()

        pts = []
        for j in range(0,len(x_WS)):
                for k in range(0,len(x_WS[j])):
                        x = x_WS[j][k].value
                        pts.append(x)

        pts = np.array(pts)
        pickle.dump( pts, open( env_folder+"/footsteppath"+str(i)+".dat", "wb" ) )
