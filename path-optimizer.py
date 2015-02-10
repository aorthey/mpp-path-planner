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
from mathtools.walkable import WalkableSurface, WalkableSurfacesFromPolytopes
from robot.htoq import *
from robot.robotspecifications import * 
from pathplanner.connectorsGetMiddlePath import * 
from pathplanner.connectorComputeNormal import * 
from pathplanner.surfaceMiddlePath import * 

from multiprocessing import Pool


###############################################################################
# changeable parameters and start/goal configs
###############################################################################
DEBUG=1
start=np.array((-0.5,2.0,0.05))
goal=np.array((0.5,-2.0,0.05))

v2 = np.array((0,0,1))
v1 = np.array((1,0,0))
v3 = np.array((0,1,0))

svLeftColor = (0.5,0,0.5,1)
svRightColor = (0.5,0,0,1)
colorScene=(0.6,0.6,0.6,0.2)
svPointSize=50

### start/goal direction
startNormal = np.array((1,-1,0))
goalNormal = np.array((0,-1,0))

###############################################################################
# preloading
###############################################################################
plot=Plotter()
robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"
env_folder = os.environ["MPP_PATH"]+"mpp-environment/output"

#path_candidates = pickle.load( open( env_folder+"/paths.dat", "rb" ) )
Wsurfaces_candidates = pickle.load( open( env_folder+"/wsurfaces.dat", "rb" ) )
Wsurfaces_Vstack_candidates = pickle.load( open( env_folder+"/wsurfaces_vstack.dat", "rb" ) )
Connector_candidates= pickle.load( open( env_folder+"/connector.dat", "rb" ) )
Connector_Vstack_candidates= pickle.load( open( env_folder+"/connector_vstack.dat", "rb" ) )
G_S = pickle.load( open( env_folder+"/graph.dat", "rb" ) )

startNormalNormal = np.dot(rotFromRPY(0,0,pi/2),startNormal)
goalNormalNormal = np.dot(rotFromRPY(0,0,pi/2),goalNormal)

heights=[]
for i in range(0,XSPACE_DIMENSION):
        heights.append(i*VSTACK_DELTA)

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
path = paths[1]

Wsurfaces=[]
Wsurfaces_Vstack=[]
Connectors=[]
Connectors_Vstack=[]
print "choosing path",path

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

N_walkablesurfaces=len(path)

###############################################################################
# obtaining middle path
###############################################################################

pathPlanes = getSurfaceMiddlePathPlanes(start, startNormal, goal, goalNormal, Wsurfaces, Connectors)
#plotMiddlePath(plot, pathPlanes)
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
constraints = Cfactory.getConstraints()
print Cfactory

###############################################################################
# building objective function
###############################################################################
Ofactory = CVXObjectiveFactory()
Ofactory.addInterpointMinimization(x_WS)
objfunc = Ofactory.getObjectiveFunction()
print Ofactory

###############################################################################
# solving problem
###############################################################################

start = timer()

rho=[]
ibRho = []

def solveMinima(i):
        print "starting minima",i,"/",XspaceMinima,"..."
        Ae = Aflat[i]
        be = bflat[i]
        mincon = []
        Ark = Aleftrightconv[i]

        rho=[]
        for j in range(0,N_walkablesurfaces-1):
                rho.append(Variable(XSPACE_DIMENSION))

        ## how many dimensions are there until the linear subspace starts?
        maxNonzeroDim = np.max(np.nonzero(Ark)[0])

        ## constraint: all intersection points have to be inside of an environment box
        for j in range(0,N_walkablesurfaces-1):
                ## constraint: points only from manifold flat inside X
                mincon.append( np.matrix(Ae)*rho[j] <= be)
                Ebox = Connectors_Vstack[j][0]
                xj = x_WS[j+1][0]
                for k in range(0,maxNonzeroDim):
                        vv = xj + rho[j][k]*v1 + heights[k]*v2
                        mincon.append( np.matrix(Ebox[k].A)*vv <= Ebox[k].b)
                        rhoR = np.matrix(Ark)*rho[j]
                        vvR = xj + rhoR[k]*v1 + heights[k]*v2
                        mincon.append( np.matrix(Ebox[k].A)*vvR <= Ebox[k].b)

        for j in range(0,len(constraints)):
                mincon.append(constraints[j])

        ctr=0
        ibRho = []

        for j in range(0,N_walkablesurfaces):
                W = Wsurfaces_Vstack[j]
                ibRho_tmp=[]

                for p in range(0,len(x_WS[j])):
                        ibRho_tmp.append(Variable(XSPACE_DIMENSION))

                for p in range(0,len(x_WS[j])):
                        mincon.append( np.matrix(Ae)*ibRho_tmp[p] <= be)
                        [a,b,ar,xcur] = pathPlanes[j][p]
                        ctr+=1
                        mincon.append(np.matrix(a).T*x_WS[j][p] == b)
                        for k in range(0,maxNonzeroDim):
                                vv = x_WS[j][p] + ibRho_tmp[p][k]*ar + heights[k]*v2
                                mincon.append( np.matrix(W[k][0].A)*vv <= W[k][0].b)
                                mincon.append( np.matrix(a).T*vv == b)
                                rhoR = np.matrix(Ark)*ibRho_tmp[p]
                                vvR = x_WS[j][p] + rhoR[k]*ar + heights[k]*v2
                                mincon.append( np.matrix(W[k][0].A)*vvR <= W[k][0].b)
                                mincon.append( np.matrix(a).T*vvR == b)

                ibRho.append(ibRho_tmp)
        ###############################################################################
        # solve
        ###############################################################################
        objective = Minimize(objfunc)
        prob = Problem(objective, mincon)
        startopt = timer()
        #prob.solve(solver=SCS, use_indirect=True, eps=1e-2, verbose=True)
        prob.solve(solver=SCS, eps=1e-3)
        ###############################################################################
        # output
        ###############################################################################

        endopt = timer()
        ts= np.around(endopt - startopt,2)
        tstotal= np.around(endopt - start,2)
        print "minima",i,"/",XspaceMinima," => ",prob.value,"(time:",ts,"s)"
        bestMinima = i
        bestMinimaValue = prob.value
        return prob.value

pool = Pool(processes = 8)
minimaArray = np.arange(0,XspaceMinima)
print minimaArray
#minimaArray = [45]

outputValues = pool.map(solveMinima, minimaArray)
#outputValues = solveMinima(45)
end = timer()
ts= np.around(end - start,2)

print "================================================================"
print "Time elapsed for checking all",XspaceMinima,"minima"
print "================="
print ts,"s"
print "================================================================"
###############################################################################
# statistics
###############################################################################
inf = float('inf')
#
validMinima = np.sum(np.array(outputValues) < inf)

pp = float(validMinima)/float(XspaceMinima)

bestMinima = np.argmin(outputValues)
bestMinimaValue = np.min(outputValues)
bestMinima = minimaArray[bestMinima]

print validMinima,"of",XspaceMinima,"are valid (",pp*100,"%)"
print "best minima:",bestMinima,"with value",bestMinimaValue

solveMinima(bestMinima)

###############################################################################
# plot
###############################################################################
if bestMinimaValue < inf:
        print "plotting workspace swept volume approximation"
        Ark = Aleftrightconv[bestMinima]
        N = len(x_WS)
        M = len(x_WS[N-1])
        x_goal = x_WS[N-1][M-1]
        x_start = x_WS[0][0]
        plot.point(x_goal.value,color=(0,0,0,0.9))
        plot.point(x_start.value)
        maxNonzeroDim = np.max(np.nonzero(Ark)[0])
        firstIS = None
        lastIS = None

        ## plot intersection environment boxes
        for i in range(0,N_walkablesurfaces-1):
                V = Connectors_Vstack[i][0]
                Nstack = Connectors_Vstack[i][1]
                for k in range(0,Nstack):
                        VV = V[k]
                        plot.polytopeFromVertices(\
                                        VV.getVertexRepresentation(),\
                                        fcolor=colorScene)

        ### plot paths on each WS
        ###  build one path for each dimension:
        svPathPoints = 0
        for i in range(0,N_walkablesurfaces):
                svPathPoints = svPathPoints + len(x_WS[i])

        svPathsLeft = np.zeros((maxNonzeroDim, svPathPoints, 3))
        svPathsRight = np.zeros((maxNonzeroDim, svPathPoints, 3))
        svPathsMiddle = np.zeros((maxNonzeroDim, svPathPoints, 3))
        thetaV = np.zeros((svPathPoints, 5))
        for k in range(0,maxNonzeroDim):
                ctr = 0
                for i in range(0,N_walkablesurfaces):
                        for j in range(0,len(x_WS[i])):
                                svPathsMiddle[k][ctr] = x_WS[i][j].value.T
                                [a,b,ar,xcur] = pathPlanes[i][j]

                                pt = x_WS[i][j].value+(ibRho[i][j][k].value*ar.T+heights[k]*v2).T
                                svPathsLeft[k][ctr] = np.array(pt).flatten()
                                ibRhoR = np.matrix(Ark)*ibRho[i][j].value
                                pt = x_WS[i][j].value+(np.array(ibRhoR[k]).flatten()[0]*ar.T+heights[k]*v2).T
                                svPathsRight[k][ctr] = np.array(pt).flatten()
                                ctr = ctr+1 
        ctr=0
        for i in range(0,N_walkablesurfaces):
                for j in range(0,len(x_WS[i])):
                        [k,h1,h2,h3] = Harray[bestMinima]
                        thetaV[ctr] = htoq(k,h1,h2,h3)[1]
                        ctr = ctr+1 

        output_folder = os.environ["MPP_PATH"]+"mpp-environment/output/"
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        svLeftFname  =   output_folder+"/xpathL.dat"
        svRightFname =   output_folder+"/xpathR.dat"
        svMiddleFname =  output_folder+"/xpathM.dat"
        svQValuesFname = output_folder+"/xpathQ.dat"

        pickle.dump( svPathsLeft, open( svLeftFname, "wb" ) )
        pickle.dump( svPathsRight, open( svRightFname, "wb" ) )
        pickle.dump( svPathsMiddle, open( svMiddleFname, "wb" ) )
        pickle.dump( thetaV, open( svQValuesFname, "wb" ) )
        plot.lines(svPathsLeft,'-or')
        plot.lines(svPathsRight,'-om')
        plot.lines(svPathsMiddle,'-ok')
        plot.set_view(90,0)
        plot.showEnvironment()

else:
        print "problem not feasible"
