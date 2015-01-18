from cvxpy import *
import cvxpy as cvx
import numpy as np
import sys
from math import pi
from mathtools.linalg import *
from mathtools.plotter import rotFromRPY
from mathtools.functional_basis import *

M = 100
t = np.linspace(0,1,M)
F = Fpoly(t)
## input: 
##      I1,I2   intersections as polytopes 
##      W       the walkable surface inbetween I1,I2
##      N       number of points on top of W to optimize
## output: 
##      X       middle path on top of W
def getMiddlePath(I1,I2,n1,n2,W,N):
        V1 = I1.getVertexRepresentation()
        V2 = I2.getVertexRepresentation()
        m1 = getMeanFromVerticesList(V1).flatten()
        m2 = getMeanFromVerticesList(V2).flatten()
        return getMiddlePathFromIntersectionVertices(m1,m2,n1,n2,W,N)
def getMiddlePathStart(p1,I2,n1,n2,W,N):
        V2 = I2.getVertexRepresentation()
        m2 = getMeanFromVerticesList(V2).flatten()
        return getMiddlePathFromIntersectionVertices(p1,m2,n1,n2,W,N)

def getMiddlePathGoal(I1,p2,n1,n2,W,N):
        V1 = I1.getVertexRepresentation()
        m1 = getMeanFromVerticesList(V1).flatten()
        return getMiddlePathFromIntersectionVertices(m1,p2,n1,n2,W,N)


def getMiddlePathFromIntersectionVertices(m1,m2,n1,n2,W,N):
        VW = W.getVertexRepresentation()
        ## compute geometrical middle
        p1 = projectPointOntoWalkableSurface(m1, W).flatten().T
        p2 = projectPointOntoWalkableSurface(m2, W).flatten().T
        print "middle points:",p1,p2

        ## set up optimization problem
        constraints = []
        objfunc = 0

        x=[]
        for i in range(0,N):
                x.append(Variable(3,1))

        constraints.append( x[0] == p1 )
        constraints.append( x[N-1] == p2 )

        ###############################################################################
        ### constraints on functional space
        ###############################################################################
        Fweight = Variable(M,3)
        if N >= M:
                print "functional space not big enough",M,"<",N
                sys.exit(0)

        for i in range(0,N):
                constraints.append( x[i] == Fweight.T*F[i] )

        objfunc += norm(Fweight)
        ###############################################################################
        gamma1 = Variable(1,1)
        constraints.append( x[1] == gamma1*n1 + x[0] )
        gamma2 = Variable(1,1)
        constraints.append( x[N-2] == gamma2*n2 + x[N-1] )

        #constraints.append( gamma1 > 0)
        #constraints.append( gamma2 < 0)
        for i in range(0,N):
                constraints.append( np.matrix(W.A)*x[i] <= W.b)
                constraints.append( np.matrix(W.ap)*x[i] == W.bp)

        for i in range(0,N-1):
                constraints.append( norm(x[i]-x[i+1]) < 0.12)

        for i in range(0,N):
                for j in range(0,len(VW)):
                        objfunc += norm(x[i]-VW[j])

        ###############################################################################
        # solve
        ###############################################################################
        objective = Minimize(objfunc)
        prob = Problem(objective, constraints)
        prob.solve(solver=cvx.SCS)

        if prob.value < float('inf'):
                xout = []
                print prob.value
                for i in range(0,N):
                        xout.append(x[i].value)
                return xout
        else:
                print "could not find middle path for WS:",W
                print x,objfunc
                sys.exit(0)

def middlePathToHyperplane(X):
        hyperplanes = []
        for i in range(0,len(X)-1):
                xcur = X[i]
                xnext = X[i+1]
                a = xnext-xcur
                a = a/np.linalg.norm(a)
                b = np.dot(xcur.T,a)
                R = rotFromRPY(0,0,pi/2)
                ar = np.dot(R,a)
                hyperplanes.append([a,b,ar,xcur])
        b = np.dot(xnext.T,a)
        hyperplanes.append([a,b,ar,xnext])
        return hyperplanes
                
