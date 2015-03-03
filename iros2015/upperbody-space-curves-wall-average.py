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
from environment.walkableSurfacesPlotter import *
from environment.scenePlotter import *
from environment.connectorsPlotter import *
from mathtools.linalg import *
from mathtools.walkable import WalkableSurface, WalkableSurfacesFromPolytopes
from robot.htoq import *
from robot.robotspecifications import * 
from pathplanner.connectorsGetMiddlePath import * 
from pathplanner.connectorComputeNormal import * 
from pathplanner.surfaceMiddlePath import * 
from pathplanner.solvePolytopeMinima import * 

import parmap

###############################################################################
# changeable parameters and start/goal configs
###############################################################################
DEBUG=0
#start=np.array((-0.2,2.4,0.05))
#goal=np.array((0.1,-2.0,0.05))
start=np.array((0.1,2.0,0.05))
goal=np.array((0.1,-2.0,0.05))

v2 = np.array((0,0,1))
v1 = np.array((1,0,0))
v3 = np.array((0,1,0))

### start/goal direction
#startNormal = np.array((1,-1,0))
#goalNormal = np.array((0,-1,0))
startNormal = np.array((0,-1,0))
goalNormal = np.array((0,-1,0))

###############################################################################
# preloading
###############################################################################

robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"
env_folder = os.environ["MPP_PATH"]+"mpp-environment/output"
fname = os.environ["MPP_PATH"]+"mpp-environment/urdf/wall-extension.urdf"

Wsurfaces_candidates = pickle.load( open( env_folder+"/wsurfaces.dat", "rb" ) )
Wsurfaces_Vstack_candidates = pickle.load( open( env_folder+"/wsurfaces_vstack.dat", "rb" ) )
Connector_candidates= pickle.load( open( env_folder+"/connector.dat", "rb" ) )
Connector_Vstack_candidates= pickle.load( open( env_folder+"/connector_vstack.dat", "rb" ) )
G_S = pickle.load( open( env_folder+"/graph.dat", "rb" ) )

startNormalNormal = np.dot(rotFromRPY(0,0,pi/2),startNormal)
goalNormalNormal = np.dot(rotFromRPY(0,0,pi/2),goalNormal)

Aname =   robot_folder+"/A.dat"
ARKname = robot_folder+"/Ark.dat"
bname =   robot_folder+"/b.dat"
HName =   robot_folder+"/H.dat"

Harray = pickle.load( open( HName, "rb" ) )
Aflat = pickle.load( open( Aname, "rb" ) )
Aleftrightconv = pickle.load( open( ARKname, "rb" ) )
bflat = pickle.load( open( bname, "rb" ) )

XspaceMinima = len(Aflat)

###############################################################################
# compute nearest walkablesurfaces from the given goal and start configuration
###############################################################################


avgtime = 0.0
Nruns=10
for avg in range(0,Nruns):
        startWS = -1
        goalWS = -1
        ds = 10000
        dg = 10000
        for i in range(0,len(Wsurfaces_candidates)):
                W = Wsurfaces_candidates[i]
                dds = distancePointWalkableSurface(start, W)
                ddg = distancePointWalkableSurface(goal, W)
                if dds < ds:
                        ds = dds
                        startWS = i
                if ddg < dg:
                        dg = ddg
                        goalWS = i

        print "goal on WS",goalWS,"(dist",np.around(dg,3),")"
        print "start on WS",startWS,"(dist",np.around(ds,3),")"

        paths=list(nx.all_simple_paths(G_S, source=startWS, target=goalWS))

        indices = []
        print "----------------------------------------------------------------"
        print "found",len(paths),"paths between walkable surfaces:"

        for i in range(0,len(paths)):
                print paths[i]
                for k in range(0,len(paths[i])):
                        indices.append(paths[i][k])

        indices = sorted(set(indices))

        print "----------------------------------------------------------------"
        print "relevant walkable surfaces:", indices
        print "----------------------------------------------------------------"

        ###############################################################################
        # Check which path has the best chances of finding a feasible trajectory
        ###############################################################################
        outputValues=[]
        outputTimes=[]
        for pathid in range(0,len(paths)):
                print "Optimization in Homotopy",pathid
                path = paths[pathid]

                Wsurfaces=[]
                Wsurfaces_Vstack=[]
                Connectors=[]
                Connectors_Vstack=[]

                for i in range(0,len(path)):
                        p = path[i]
                        Wsurfaces.append(Wsurfaces_candidates[p])
                        Wsurfaces_Vstack.append(Wsurfaces_Vstack_candidates[p])
                        print len(Wsurfaces_Vstack_candidates[p])

                for i in range(0,len(path)-1):
                        p = path[i]
                        pn = path[i+1]
                        for j in range(0,len(Connector_candidates)):
                                cp = Connector_candidates[j][1]
                                cpn = Connector_candidates[j][2]
                                if (cp == p and cpn == pn) or (cp == pn and cpn == p):
                                        Connectors_Vstack.append(Connector_Vstack_candidates[j])
                                        Connectors.append(Connector_candidates[j])
                                        break

                for i in range(0,len(Connectors)):
                        Nv = Connectors_Vstack[i][1]
                        p  = Connectors_Vstack[i][2]
                        pn = Connectors_Vstack[i][3]
                        print "connection",p,"to",pn,"has",Nv,"layers"

                N_walkablesurfaces = len(path)

                ###############################################################################
                # obtaining middle path
                ###############################################################################
                for i in range(len(Wsurfaces)):
                        V = Wsurfaces[i].getVertexRepresentation()
                        plot.walkableSurface( V,\
                                        fcolor=COLOR_SCENE, thickness=0.01)

                pathPlanes = getSurfaceMiddlePathPlanes(start, startNormal, goal, goalNormal, Wsurfaces, Connectors)

                output_folder = os.environ["MPP_PATH"]+"mpp-path-planner/output/homotopy"+str(pathid)
                if not os.path.exists(output_folder):
                    os.makedirs(output_folder)
                pickle.dump( pathPlanes, open( output_folder+"/pathplanes.dat", "wb" ) )

                if DEBUG:
                        plotMiddleFootpath(plot, pathPlanes, start)
                        plotMiddlePath(plot, pathPlanes)
                        plot.set_view(57,39)
                        plot.ax.set_xlim(-1,1)
                        plot.ax.set_ylim(-3,3)
                        plot.ax.set_zlim(0,1)
                        plot.showEnvironment()
                        sys.exit(0)

                x_WS = pathPlanesToCVXfootVariables(pathPlanes)

                ###############################################################################
                # building constraints
                ###############################################################################
                from pathplanner.cvxConstraintFactory import CVXConstraintFactory
                from pathplanner.cvxObjectiveFactory import CVXObjectiveFactory

                Cfactory = CVXConstraintFactory(N_walkablesurfaces)
                Cfactory.addDistanceBetweenFootpoints(x_WS)
                Cfactory.addStartPosFootpoint(x_WS,start,Wsurfaces)
                Cfactory.addGoalPosFootpoint(x_WS,goal,Wsurfaces)
                Cfactory.addFootpointOnSurfaceConstraint(x_WS, Wsurfaces)
                Cfactory.addConnectionConstraintsOnFootpoints(x_WS, Connectors)
                Cfactory.addMiddlePathHyperplaneOnFootpoints(x_WS, pathPlanes)
                Cfactory.addStartNormalConstraint(x_WS, startNormal)
                Cfactory.addGoalNormalConstraint(x_WS, goalNormal)
                Cfactory.addFootpointPerpendicularConstraints(x_WS, Connectors)

                constraints = Cfactory.getConstraints()
                print Cfactory

                ###############################################################################
                # building objective function
                ###############################################################################
                Ofactory = CVXObjectiveFactory()
                Ofactory.addInterpointMinimization(x_WS)
                Ofactory.addLineMinimization(x_WS,np.array((0,1,0)) )
                objfunc = Ofactory.getObjectiveFunction()
                print Ofactory

                ###############################################################################
                # solving problem
                ###############################################################################
                ptimer = Timer()
                pool = Pool(processes = 8)
                minimaArray = np.arange(0,XspaceMinima)

                outputValues.append(parmap.map(solvePolytopeMinima, minimaArray, \
                                pathid, Aflat, bflat, Harray,\
                                Aleftrightconv, constraints, objfunc, \
                                Wsurfaces_Vstack, Connectors_Vstack, x_WS, \
                                SV_HEIGHTS, pathPlanes))

                ptimer.stop()
                outputTimes.append(ptimer.getTime())

        ###############################################################################
        # statistics
        ###############################################################################
        avgtimer.stop()
        avgtime+=avgtimer.getTime()/float(Nruns)
print "========================================================================"
print "Computing all minima (average over",Nruns,")"
print avgtime,"s"
print "========================================================================"
