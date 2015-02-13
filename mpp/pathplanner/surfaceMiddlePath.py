from cvxpy import *
import cvxpy as cvx
import numpy as np
import sys
from math import pi
from robot.robotspecifications import *
from mathtools.util import *
from pathplanner.connectorsGetMiddlePath import * 
from pathplanner.connectorComputeNormal import * 

def getSurfaceMiddlePathPlanes(start, startNormal, goal, goalNormal, Wsurfaces, Connectors):
        ### we have to know the intersection normals, and the normal to those normals
        connectorNormalNormal = []
        connectorNormal= []

        for i in range(0,len(Wsurfaces)-1):
                I = Connectors[i][0]
                [n,nn]=computeNormalFromIntersection(I)
                connectorNormalNormal.append(nn)
                connectorNormal.append(n)

        ###############################################################################
        ## compute number of points per walkable surface, depending on the distance to
        ## the next connector segment
        pathPlanes = []

        W=Wsurfaces[0]
        dI = distancePointWalkableSurface(start, Connectors[0][0])
        NdI = distanceInMetersToNumberOfPoints(dI)
        Xmiddle = getMiddlePathStart(start,Connectors[0][0],startNormal,connectorNormal[0],W,NdI)
        pathPlanes.append(middlePathToHyperplane(Xmiddle))

        #pathPlanesStart = middlePathToHyperplane(Xmiddle)

        #pathPlanes_tmp = []
        #for i in range(0,len(Xmiddle)):
        #        pathPlanes_tmp.append(pathPlanesStart[i])


        #pathPlanesConnector = []

        for i in range(0,len(Connectors)-1):
                C = Connectors[i][0]
                Cnext = Connectors[i+1][0]
                dcc = distanceWalkableSurfaceWalkableSurface(C,Cnext)
                Ndcc = distanceInMetersToNumberOfPoints(dcc)
                #M_w.append(Ndcc)
                W=Wsurfaces[i+1]
                Xmiddle = getMiddlePath(C,Cnext,connectorNormal[i],connectorNormal[i+1],W,Ndcc)
                pathPlanes.append( middlePathToHyperplane(Xmiddle) )

        #for i in range(0,len(pathPlanesConnector)):
        #        for j in range(0,len(pathPlanesConnector[i])):
        #                pathPlanes.append(pathPlanesConnector[i][j])

        NC = len(Connectors)-1
        dG = distancePointWalkableSurface(goal, Connectors[NC][0])
        Ndg = distanceInMetersToNumberOfPoints(dG)
        #M_w.append(Ndg)

        W=Wsurfaces[len(Wsurfaces)-1]
        Xmiddle = getMiddlePathGoal(Connectors[NC][0],goal,connectorNormal[NC],goalNormal,W,Ndg)
        #pathPlanesGoal = middlePathToHyperplane(Xmiddle)
        pathPlanes.append(middlePathToHyperplane(Xmiddle))

        #for i in range(0,len(pathPlanesGoal)):
                #pathPlanes.append(pathPlanesGoal[i])

        print "computed middle paths"
        return pathPlanes

def plotMiddlePath(plot, pathPlanes):
        for i in range(0,len(pathPlanes)):
                for j in range(0,len(pathPlanes[i])):
                        zz=np.array((0,0,1))
                        [a,b,ar,xcur]=pathPlanes[i][j]
                        ## zd: height of cross section
                        ## xd: width of cross section
                        zd = 1.1
                        xd = 1.2

                        l1 = xcur-0.5*xd*ar
                        l2 = xcur+0.5*xd*ar
                        l3 = xcur
                        L = np.array([l1,l2,l3])
                        #plot.point(l1)
                        #plot.point(l2,color=COLOR_SWEPTVOLUME_LEFT)
                        #plot.point(l3,color=COLOR_SWEPTVOLUME_RIGHT)
                        #plot.line(L,lw=0.3)

                        l4 = xcur-0.5*xd*ar
                        l5 = xcur+0.5*xd*ar
                        l6 = xcur
                        l4[2]=l1[2]+zd
                        l5[2]=l2[2]+zd
                        l6[2]=l3[2]+zd

                        dd=0.02
                        l7 = l4+dd*a
                        l8 = l5+dd*a
                        l9 = l6+dd*a

                        l10 = l1+dd*a
                        l11 = l2+dd*a
                        l12 = l3+dd*a

                        V=[]
                        V.append(l1)
                        V.append(l2)
                        V.append(l3)
                        V.append(l4)
                        V.append(l5)
                        V.append(l6)
                        V.append(l7)
                        V.append(l8)
                        V.append(l9)
                        V.append(l10)
                        V.append(l11)
                        V.append(l12)

                        V=np.array(V)
                        V=V.T[0]
                        plot.polytopeFromVertices(V.T,fcolor=COLOR_CROSS_SECTION)

def pathPlanesToCVXfootVariables(pathPlanes):
        x_WS = []
        for i in range(0,len(pathPlanes)):
                M = len(pathPlanes[i])
                print "WS ",i,"has",M,"points to optimize"
                x_WS_tmp = []
                for j in range(0,M):
                        x_WS_tmp.append(Variable(3,1))
                x_WS.append(x_WS_tmp)
        return x_WS
