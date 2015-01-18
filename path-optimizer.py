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
plot=Plotter()

robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"
env_folder = os.environ["MPP_PATH"]+"mpp-environment/output"

#path_candidates = pickle.load( open( env_folder+"/paths.dat", "rb" ) )
Wsurfaces_candidates = pickle.load( open( env_folder+"/wsurfaces.dat", "rb" ) )
Wsurfaces_Vstack_candidates = pickle.load( open( env_folder+"/wsurfaces_vstack.dat", "rb" ) )
Connector_candidates= pickle.load( open( env_folder+"/connector.dat", "rb" ) )
Connector_Vstack_candidates= pickle.load( open( env_folder+"/connector_vstack.dat", "rb" ) )
G_S = pickle.load( open( env_folder+"/graph.dat", "rb" ) )

DEBUG=1

start=np.array((-0.5,2.0,0.05))
goal=np.array((0.0,-2.0,0.05))

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
# building variables
###############################################################################
## the x positions on the connection between two walkable surfaces
x_goal = Variable(3,1)
x_start = Variable(3,1)

x_connection = []
for i in range(0,N_walkablesurfaces-1):
        x_connection.append(Variable(3,1))

###############################################################################
### we have to know the intersection normals, and the normal to those normals
connectorNormalNormal = []
connectorNormal= []
for i in range(0,N_walkablesurfaces-1):
        I = Connectors[i][0]
        [n,nn]=computeNormalFromIntersection(I)
        connectorNormalNormal.append(nn)
        connectorNormal.append(n)

###############################################################################
## compute number of points per walkable surface, depending on the distance to
## the next connector segment
M_w = []

N_c = len(Connectors)
dI = distancePointWalkableSurface(start, Connectors[0][0])
NdI = distanceInMetersToNumberOfPoints(dI)

pathPlanes = []
W=Wsurfaces[0]
Xmiddle = getMiddlePathStart(start,Connectors[0][0],startNormal,connectorNormal[0],W,NdI)
print Xmiddle[0],Xmiddle[-1]

pathPlanesStart = middlePathToHyperplane(Xmiddle)

for i in range(0,len(Xmiddle)):
        pathPlanes.append(pathPlanesStart[i])

M_w.append(NdI)
pathPlanesConnector = []

for i in range(0,N_c-1):
        C = Connectors[i][0]
        Cnext = Connectors[i+1][0]
        dcc = distanceWalkableSurfaceWalkableSurface(C,Cnext)
        Ndcc = distanceInMetersToNumberOfPoints(dcc)
        M_w.append(Ndcc)
        W=Wsurfaces[i+1]
        Xmiddle = getMiddlePath(C,Cnext,connectorNormal[i],connectorNormal[i+1],W,Ndcc)
        pathPlanesConnector.append( middlePathToHyperplane(Xmiddle) )
        print Xmiddle[0],Xmiddle[-1]

for i in range(0,len(pathPlanesConnector)):
        for j in range(0,len(pathPlanesConnector[i])):
                pathPlanes.append(pathPlanesConnector[i][j])

dG = distancePointWalkableSurface(goal, Connectors[N_c-1][0])
Ndg = distanceInMetersToNumberOfPoints(dG)
M_w.append(Ndg)

W=Wsurfaces[len(path)-1]
Xmiddle = getMiddlePathGoal(Connectors[N_c-1][0],goal,connectorNormal[N_c-1],goalNormal,W,Ndg)
print Xmiddle[0],Xmiddle[-1]
pathPlanesGoal = middlePathToHyperplane(Xmiddle)

for i in range(0,len(pathPlanesGoal)):
        pathPlanes.append(pathPlanesGoal[i])

###############################################################################
## DEBUG plot
xx = (start.T+np.array((0,0,0.1))).T
plot.point(xx,color=(0,0,0,1))
x_WS = []
for i in range(0,N_walkablesurfaces):
        print "WS ",i,"has",M_w[i],"points to optimize"
        x_WS_tmp = []
        for j in range(0,M_w[i]):
                x_WS_tmp.append(Variable(3,1))
        x_WS.append(x_WS_tmp)

ctr=0
for i in range(0,N_walkablesurfaces):
        print "surface",i
        for j in range(0,M_w[i]):
                [a,b,ar,xcur]=pathPlanes[ctr]
                l1 = xcur-0.5*ar
                l2 = xcur+0.5*ar
                l3 = xcur
                L = np.array([l1,l2,l3])
                plot.point(l1)
                plot.point(l2,color=(1,0,1,1))
                plot.point(l3,color=(0,0,0,1))
                plot.line(L,lw=0.3)
                ctr=ctr+1

for i in range(len(Wsurfaces)):
        V = Wsurfaces[i].getVertexRepresentation()
        plot.walkableSurface( V,\
                        fcolor=((0.2,0.2,0.2,0.5)), thickness=0.01)

#plot.show()
#sys.exit(0)
###############################################################################
# building constraints
###############################################################################
constraints = []
objfunc = 0

## start/goal regions constraints
constraints.append(norm(x_start - start) <= PATH_RADIUS_START_REGION)
constraints.append(norm(x_goal - goal) <= PATH_RADIUS_GOAL_REGION)

W=Wsurfaces[0]
constraints.append( np.matrix(W.A)*x_start <= W.b )
constraints.append( np.matrix(W.ap)*x_start == W.bp)
W=Wsurfaces[len(path)-1]
constraints.append( np.matrix(W.A)*x_goal <= W.b )
constraints.append( np.matrix(W.ap)*x_goal == W.bp)

###############################################################################
for i in range(0,N_walkablesurfaces-1):
        ### constraints for points on the connection of the WS
        C = Connectors[i][0]
        y = x_connection[i]
        constraints.append( np.matrix(C.A)*y <= C.b )
        constraints.append( np.matrix(C.ap)*y == C.bp)

###############################################################################
for i in range(0,N_walkablesurfaces):
        #### x_WS should contain only points on the surface
        W=Wsurfaces[i]
        plot.walkableSurface( \
                        W.getVertexRepresentation(),\
                        fcolor=colorScene,\
                        thickness=0.01)
        for j in range(0,M_w[i]):
                constraints.append( np.matrix(W.A)*x_WS[i][j] <= W.b)
                constraints.append( np.matrix(W.ap)*x_WS[i][j] == W.bp)

###############################################################################
ctr=0
for i in range(0,N_walkablesurfaces):
        ### constraints for points being on the hyperplanes of the middle path
        for j in range(0,len(x_WS[i])):
                [a,b,ar,xcur] = pathPlanes[ctr]
                constraints.append(np.matrix(a).T*x_WS[i][j] == b)
                ctr+=1

###############################################################################
### constraint: x_WS should lie in the same functional space

M = 100
t = np.linspace(0,1,M)
F = Fpoly(t)
for i in range(0,N_walkablesurfaces):
        ### constraints for points being on the hyperplanes of the middle path
        N = len(x_WS[i])
        Fweight = Variable(M,3)
        if N >= M:
                print "functional space not big enough",M,"<",N
                sys.exit(0)

        for j in range(0,N):
                constraints.append( x_WS[i][j] == Fweight.T*F[j] )

        objfunc += norm(Fweight)

###############################################################################
### constraint: x_WS connections on connectors
constraints.append( x_WS[0][0] == x_start)
constraints.append( x_WS[0][M_w[0]-1] == x_connection[0])

for i in range(1,N_walkablesurfaces-1):
        Lws = len(x_WS[i])-1
        constraints.append( x_WS[i][0] == x_connection[i-1])
        constraints.append( x_WS[i][Lws] == x_connection[i])

Lws = len(x_WS[N_walkablesurfaces-1])-1
constraints.append( x_WS[N_walkablesurfaces-1][0] == x_connection[N_walkablesurfaces-2])
constraints.append( x_WS[N_walkablesurfaces-1][Lws] == x_goal)

## catch the imprecision of the previous cvx solver
#precision = 0.001
#constraints.append( norm(x_WS[0][0] - x_start) <= precision)
#constraints.append( norm(x_WS[0][M_w[0]-1] - x_connection[0]) <= precision)
#
#for i in range(1,N_walkablesurfaces-1):
#        Lws = len(x_WS[i])-1
#        constraints.append( norm(x_WS[i][0] - x_connection[i-1]) <= precision)
#        constraints.append( norm(x_WS[i][Lws] - x_connection[i]) <= precision)
#
#Lws = len(x_WS[N_walkablesurfaces-1])-1
#constraints.append( norm(x_WS[N_walkablesurfaces-1][0] - x_connection[N_walkablesurfaces-2]) <= precision)
#constraints.append( norm(x_WS[N_walkablesurfaces-1][Lws] - x_goal) <= precision)

###############################################################################
### constraint: distance between x_WS
for i in range(0,N_walkablesurfaces):
        for j in range(1,len(x_WS[i])):
                constraints.append(norm(x_WS[i][j] - x_WS[i][j-1]) <= PATH_DIST_WAYPOINTS_MAX)

###############################################################################
### constraint: x_WS before the intersection should have the same orientation as
### the intersection

for i in range(0,N_walkablesurfaces-1):
        v = connectorNormal[i]
        Lws = len(x_WS[i])-1
        ## trois points: x before the intersection, x at the intersection and x
        ## after the intersection. They all should lie on a line perpendicular to
        ## the normalnormal of the intersection
        xbefore = x_WS[i][Lws-1]
        xconnect = x_WS[i+1][0]
        xafter = x_WS[i+1][1]

        gammaA = Variable(1)
        gammaB = Variable(1)
        constraints.append( gammaA*v + xconnect == xafter )
        constraints.append( gammaB*v + xconnect == xbefore )

###############################################################################
### constraint: the first point after start should have same orientation as start
v = startNormal
xnext = x_WS[0][1]
gammaStart = Variable(1)
constraints.append( gammaStart*v + x_WS[0][0] == xnext )

### constraint: the first point before goal should have same orientation as goal
v = goalNormal
N = len(x_WS)
M = len(x_WS[N-1])
xbefore = x_WS[N-1][M-2]
xgoal = x_WS[N-1][M-1]

gammaGoal = Variable(1)
constraints.append( gammaGoal*v + xgoal == xbefore )


###############################################################################
# building objective
###############################################################################
startMinima = 27

allValuesFirst = []

start = timer()
bestMinima = 0
bestMinimaValue = inf
for i in range(startMinima,XspaceMinima):
        Ae = Aflat[i]
        be = bflat[i]
        mincon = []
        Ark = Aleftrightconv[i]
        rho = Variable(XSPACE_DIMENSION)

        ## constraint: points only from manifold flat inside X

        mincon.append( np.matrix(Ae)*rho <= be)

        ## how many dimensions are there until the linear subspace starts?
        maxNonzeroDim = np.max(np.nonzero(Ark)[0])

        ## constraint: all intersection points have to be inside of an environment box
        for j in range(0,N_walkablesurfaces-1):
                Ebox = Connectors_Vstack[j][0]
                for k in range(0,maxNonzeroDim):
                        vv = x_connection[j] + rho[k]*v1 + heights[k]*v2
                        mincon.append( np.matrix(Ebox[k].A)*vv <= Ebox[k].b)
                        rhoR = np.matrix(Ark)*rho
                        vvR = x_connection[j] + rhoR[k]*v1 + heights[k]*v2
                        mincon.append( np.matrix(Ebox[k].A)*vvR <= Ebox[k].b)

        for j in range(0,len(constraints)):
                mincon.append(constraints[j])

        ctr=0
        ibRho = []
        for j in range(0,N_walkablesurfaces):
                W = Wsurfaces_Vstack[j]
                ibRho_tmp=[]
                if j>0:
                        objfunc += norm(x_WS[j][0]-x_WS[j-1][len(x_WS[j-1])-1])


                for p in range(0,len(x_WS[j])-1):
                        objfunc += norm(x_WS[j][p]-x_WS[j][p+1])
                for p in range(0,len(x_WS[j])):
                        ibRho_tmp.append(Variable(XSPACE_DIMENSION))

                for p in range(0,len(x_WS[j])):
                        mincon.append( np.matrix(Ae)*ibRho_tmp[p] <= be)
                        [a,b,ar,xcur] = pathPlanes[ctr]
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
        prob.solve(solver=SCS, use_indirect=True, eps=1e-2)
        print "minima",i,"/",XspaceMinima," => ",prob.value
        allValuesFirst.append(prob.value)
        endopt = timer()
        ts= np.around(endopt - startopt,2)
        print "convex problem",i,"took",ts,"s"
        if prob.value < inf:
                print "minima",i,"admits a solution"
                if bestMinimaValue > prob.value:
                        bestMinimaValue = prob.value
                        bestMinima = i
                break
        if i%100==0:
                end = timer()
                ts= np.around(end - start,2)
                validMinima = np.sum(np.array(allValuesFirst) < inf)
                print "================================================================"
                print "Time elapsed after checking",i,"minima:"
                print ts,"s"
                print "(",validMinima,"valid minima found so far)"
                print "================================================================"


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

validMinima = np.sum(np.array(allValuesFirst) < inf)

pp = float(validMinima)/float(XspaceMinima)

print validMinima,"of",XspaceMinima,"are valid (",pp*100,"%)"
print "best minima:",bestMinima,"with value",bestMinimaValue

###############################################################################
# plot
###############################################################################
if prob.value < inf:
        print "plotting workspace swept volume approximation"
        Ark = Aleftrightconv[bestMinima]
        rhoR = np.matrix(Ark)*rho.value
        plot.point(x_goal.value,color=(0,0,0,0.9))
        plot.point(x_start.value)
        maxNonzeroDim = np.max(np.nonzero(Ark)[0])
        firstIS = None
        lastIS = None

        ## plot intersection environment boxes
        for i in range(0,N_walkablesurfaces-1):
                if x_connection[i].value is not None:
                        V = Connectors_Vstack[i][0]
                        for k in range(0,maxNonzeroDim):
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
                                [a,b,ar,xcur] = pathPlanes[ctr]

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
