from cvxpy import *
import cvxpy as cvx
import numpy as np
import sys
from math import pi
from copy import copy 
from robot.robotspecifications import *
from mathtools.functional_basis import *

class CVXConstraintFactory:
        constraints = []
        constraintNames = []

        def __init__(self,NwalkableSurfaces):
                self.Nws = NwalkableSurfaces
                self.constraints = []
                self.constraintNames = []

        def __str__(self):
                out = ""
                out += "------------------------------\n"
                out += "Constraint Factory\n"
                out += "------------------------------\n"
                for i in range(0,len(self.constraintNames)):
                        out+=self.constraintNames[i]
                        out+="\n"
                out += "------------------------------\n"
                out += "Total constraints: "+str(len(self.constraints))+"\n"
                out += "------------------------------\n"
                return out

        def getConstraints(self):
                return self.constraints


        ### TODO: choose appropriate number of points depending on size
        def getFootpointsFromWalkableSurfaces(self, Wsurfaces):
                M=MINIMUM_SAMPLES_PER_WALKABLE_SURFACE
                x_WS=[]
                for i in range(0,len(Wsurfaces)):
                        x_WS_tmp = []
                        W=Wsurfaces[i]
                        for j in range(0,M):
                                x_WS_tmp.append(Variable(3,1))
                        x_WS.append(x_WS_tmp)
                return x_WS

        def getFunctionalSpace(self, x_WS):
                N = 0
                for i in range(0,len(x_WS)):
                        N+=len(x_WS[i])

                M = M_TRAJECTORY_POINTS
                t = np.linspace(0,1,M)
                F = Fpoly(t)
                F=F.T
                #F = Ffourier(t)
                #F = F.T
                Fweight = Variable(M,3)
                if N >= M:
                        print "functional space not big enough",M,"<",N
                        sys.exit(0)
                return [F,Fweight]

        def addFootpointInFunctionalSpace(self, x_WS, F, W):
                ctr=0
                for i in range(0,len(x_WS)):
                        for j in range(0,len(x_WS[i])):
                                #self.constraints.append( x_WS[i][j] == W.T*F[:,ctr] )
                                self.constraints.append( norm(x_WS[i][j] - W.T*F[:,ctr])<EPSILON_FUNCTIONAL_SPACE )
                                ctr+=1
                self.constraintNames.append("footpoints inside functional space ("+str(ctr)+" constraints)")

        def addDistanceBetweenFootpoints(self,x_WS):
                assert len(x_WS)==self.Nws
                ctr = 0
                for i in range(0,self.Nws):
                        for j in range(1,len(x_WS[i])):
                                self.constraints.append(norm(x_WS[i][j] - x_WS[i][j-1]) <= PATH_DIST_WAYPOINTS_MAX)
                                ctr+=1
                self.constraintNames.append("distance between footpoints ("+str(ctr)+" constraints)")

        def addStartPosFootpoint(self, x_WS, start, W):
                Wstart = W[0]
                self.constraints.append(norm(x_WS[0][0] - start) <= PATH_RADIUS_START_REGION)
                self.constraints.append(  np.matrix(Wstart.A)*x_WS[0][0] <= Wstart.b )
                self.constraints.append( np.matrix(Wstart.ap)*x_WS[0][0] == Wstart.bp)
                self.constraintNames.append("footpoint at start position ("+str(3)+" constraints)")

        def addGoalPosFootpoint(self, x_WS, goal, W):
                Wgoal = W[len(W)-1]
                N=len(W)-1
                M=len(x_WS[N])-1
                self.constraints.append(norm(x_WS[N][M] - goal) <= PATH_RADIUS_GOAL_REGION)
                self.constraints.append(  np.matrix(Wgoal.A)*x_WS[N][M] <= Wgoal.b )
                self.constraints.append( np.matrix(Wgoal.ap)*x_WS[N][M] == Wgoal.bp)
                self.constraintNames.append("footpoint at goal position ("+str(3)+" constraints)")

        def addFootpointOnSurfaceConstraint(self, x_WS, W):
                ctr=0
                for i in range(0,len(W)):
                        Wcur=W[i]
                        for j in range(0,len(x_WS[i])):
                                self.constraints.append( np.matrix(Wcur.A)*x_WS[i][j] <= Wcur.b)
                                self.constraints.append( np.matrix(Wcur.ap)*x_WS[i][j] == Wcur.bp)
                                ctr+=2
                self.constraintNames.append("footpoint on walkable surfaces ("+str(ctr)+" constraints)")

        def addFootpointInsideSurfaceConstraint(self, x_WS, Wsurfaces):
                R=MIN_DISTANCE_POINTS_TO_BOUNDARY
                ctr=0

                for i in range(0,len(x_WS)):
                        W=Wsurfaces[i]
                        ap = W.ap
                        A = W.A
                        b = W.b
                        Bfoot = copy(b)
                        for k in range(0,len(x_WS[i])):
                                for j in range(0,W.numberOfHalfspaces()):
                                        aprime = A[j] - np.dot(A[j],ap)*ap
                                        anorm = np.linalg.norm(aprime)
                                        if anorm>0.001:
                                                aa = np.dot(A[j],aprime)
                                                Bfoot[j]=b[j]-R*aa
                                                self.constraints.append( np.matrix(A[j])*x_WS[i][k] <= Bfoot[j])
                                                ctr+=1

                self.constraintNames.append("footpoints distance from surface border ("+str(ctr)+" constraints)")


        def addConnectorVstackConstraint(self, X, CVstack):
                V=CVstack
                for i in range(0,len(X)):
                        self.constraints.append( np.matrix(V[i].A)*X[i] <= V[i].b )

                self.constraintNames.append("vstack connector constraint ("+str(len(X))+" constraints)")

        def addConnectionConstraintsOnFootpoints(self, x_WS, Connectors):
                ctr = 0
                for i in range(0,len(x_WS)-1):
                        C = Connectors[i][0]
                        yi = x_WS[i][len(x_WS[i])-1]
                        yii = x_WS[i+1][0]
                        self.constraints.append( yi == yii )
                        self.constraints.append( np.matrix(C.A)*yi <= C.b )
                        self.constraints.append( np.matrix(C.ap)*yi == C.bp)
                        self.constraints.append( np.matrix(C.A)*yii <= C.b )
                        self.constraints.append( np.matrix(C.ap)*yii == C.bp)
                        ctr+=5
                self.constraintNames.append("footpts connection between surfaces ("+str(ctr)+" constraints)")
        def addMiddlePathHyperplaneOnFootpoints(self, x_WS, pathPlanes):
                ctr=0
                for i in range(0,len(x_WS)):
                        ### constraints for points being on the hyperplanes of the middle path
                        for j in range(0,len(x_WS[i])):
                                [a,b,ar,xcur] = pathPlanes[i][j]
                                self.constraints.append(np.matrix(a).T*x_WS[i][j] == b)
                                ctr+=1
                self.constraintNames.append("footpts connection between surfaces ("+str(ctr)+" constraints)")
        def addStartNormalConstraint(self, x_WS, startNormal):
                gammaStart = Variable(1)
                self.constraints.append( gammaStart*startNormal + x_WS[0][0] == x_WS[0][1] )
                self.constraintNames.append("first two steps oriented to start ("+str(1)+" constraints)")
        def addGoalNormalConstraint(self, x_WS, goalNormal):
                v = goalNormal
                N = len(x_WS)
                M = len(x_WS[N-1])
                xbefore = x_WS[N-1][M-2]
                xgoal = x_WS[N-1][M-1]
                gammaGoal = Variable(1)
                self.constraints.append( gammaGoal*v + xgoal == xbefore )
                self.constraintNames.append("last two steps oriented to goal ("+str(1)+" constraints)")
        #def addConnectorNormalConstraints(self, x_WS, connectorNormal):
        #        ctr = 0
        #        for i in range(0,len(x_WS)-1):
        #                v = connectorNormal[i]
        #                Lws = len(x_WS[i])-1
        #                ## trois points: x before the intersection, x at the intersection and x
        #                ## after the intersection. They all should lie on a line perpendicular to
        #                ## the normalnormal of the intersection
        #                xbefore = x_WS[i][Lws-1]
        #                xconnect = x_WS[i+1][0]
        #                xafter = x_WS[i+1][1]

        #                gammaA = Variable(1)
        #                gammaB = Variable(1)
        #                self.constraints.append( gammaA*v + xconnect == xafter )
        #                self.constraints.append( gammaB*v + xconnect == xbefore )
        #                ctr+=2
        #        self.constraintNames.append("footpts orientation connection ("+str(ctr)+" constraints)")
