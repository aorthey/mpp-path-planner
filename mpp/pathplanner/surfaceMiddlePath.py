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
                        [a,b,ar,xcur]=pathPlanes[i][j]
                        l1 = xcur-0.5*ar
                        l2 = xcur+0.5*ar
                        l3 = xcur
                        L = np.array([l1,l2,l3])
                        plot.point(l1)
                        plot.point(l2,color=(1,0,1,1))
                        plot.point(l3,color=(0,0,0,1))
                        plot.line(L,lw=0.3)

        #for i in range(len(Wsurfaces)):
        #        V = Wsurfaces[i].getVertexRepresentation()
        #        plot.walkableSurface( V,\
        #                        fcolor=((0.2,0.2,0.2,0.5)), thickness=0.01)

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
