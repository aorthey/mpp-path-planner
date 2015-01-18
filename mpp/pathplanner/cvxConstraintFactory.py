from cvxpy import *
import cvxpy as cvx
import numpy as np
import sys
from math import pi
from robot.robotspecifications import *

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
                                [a,b,ar,xcur] = pathPlanes[ctr]
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
        def addConnectorNormalConstraints(self, x_WS, connectorNormal):
                ctr = 0
                for i in range(0,len(x_WS)-1):
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
                        self.constraints.append( gammaA*v + xconnect == xafter )
                        self.constraints.append( gammaB*v + xconnect == xbefore )
                        ctr+=2
                self.constraintNames.append("footpts orientation connection ("+str(ctr)+" constraints)")
