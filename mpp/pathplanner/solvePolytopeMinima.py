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


def solvePolytopeMinima(Ae,be,Ark,constraints,objfunc,Wsurfaces_Vstack,Connectors_Vstack,x_WS,heights,pathPlanes,pid):
        Z_AXIS = np.array((0,0,1))
        X_AXIS = np.array((1,0,0))

        print "starting minima",pid

        rho=[]
        mincon = []
        N_walkablesurfaces=len(Wsurfaces_Vstack)
        

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
                        ### TODO: add instead of X_AXIS the normalNormal of
                        ### connector element
                        vv = xj + rho[j][k]*X_AXIS + heights[k]*Z_AXIS
                        mincon.append( np.matrix(Ebox[k].A)*vv <= Ebox[k].b)
                        rhoR = np.matrix(Ark)*rho[j]
                        vvR = xj + rhoR[k]*X_AXIS + heights[k]*Z_AXIS
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
                                vv = x_WS[j][p] + ibRho_tmp[p][k]*ar + heights[k]*Z_AXIS
                                mincon.append( np.matrix(W[k][0].A)*vv <= W[k][0].b)
                                mincon.append( np.matrix(a).T*vv == b)
                                rhoR = np.matrix(Ark)*ibRho_tmp[p]
                                vvR = x_WS[j][p] + rhoR[k]*ar + heights[k]*Z_AXIS
                                mincon.append( np.matrix(W[k][0].A)*vvR <= W[k][0].b)
                                mincon.append( np.matrix(a).T*vvR == b)

                ibRho.append(ibRho_tmp)
        ###############################################################################
        # solve
        ###############################################################################
        objective = Minimize(objfunc)
        prob = Problem(objective, mincon)
        #prob.solve(solver=SCS, use_indirect=True, eps=1e-2, verbose=True)
        prob.solve(solver=SCS, eps=1e-3)

        ###############################################################################
        # output
        ###############################################################################

        print "minima",pid,"=> ",prob.value
        return [prob.value,pid]

