from cvxpy import *
import cvxpy as cvx
import numpy as np
import sys
from mathtools.linalg import *
from mathtools.plotter import rotFromRPY
from math import pi

def computeNormalFromIntersection(I):
        #V = I.getVertexRepresentation()
        #M = len(V)

        v1 = np.array((10,0,0))
        v2 = np.array((-10,0,0))
        v3 = np.array((0,10,0))
        v4 = np.array((0,-10,0))
        p1 = projectPointOntoWalkableSurface(v1, I)
        p2 = projectPointOntoWalkableSurface(v2, I)
        p3 = projectPointOntoWalkableSurface(v3, I)
        p4 = projectPointOntoWalkableSurface(v4, I)

        dx = np.linalg.norm(p1-p2)
        dy = np.linalg.norm(p3-p4)

        if dx>dy:
                nn = v1-v2
        else:
                nn = v3-v4

        nn=nn/np.linalg.norm(nn)
        n = np.dot(rotFromRPY(0,0,pi/2),nn)
        return [n,nn]


