import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

import numpy as np
import cvxpy as cvx
import pickle
from robot.robotspecifications import *
from robot.htoq import *

def upperBodyPathToFile(mid, Ark, x_WS, ibRho, H, pathPlanes, homotopy):
        N = len(x_WS)
        M = len(x_WS[N-1])
        x_goal = x_WS[N-1][M-1]
        x_start = x_WS[0][0]
        maxNonzeroDim = np.max(np.nonzero(Ark)[0])
        firstIS = None
        lastIS = None

        N_walkablesurfaces = len(x_WS)

        svPathPoints = 0
        for i in range(0,N_walkablesurfaces):
                svPathPoints = svPathPoints + len(x_WS[i])

        svPathsLeft = np.zeros((maxNonzeroDim, svPathPoints, 3))
        svPathsRight = np.zeros((maxNonzeroDim, svPathPoints, 3))
        svPathsMiddle = np.zeros((maxNonzeroDim, svPathPoints, 3))
        thetaV = np.zeros((svPathPoints, 5))
        footPath = np.zeros((svPathPoints, 6))

        for k in range(0,maxNonzeroDim):
                ctr = 0
                for i in range(0,N_walkablesurfaces):
                        for j in range(0,len(x_WS[i])):
                                [a,b,ar,xcur] = pathPlanes[i][j]
                                ###############################################
                                ### footpath
                                xf = x_WS[i][j].value[0]
                                yf = x_WS[i][j].value[1]
                                zf = x_WS[i][j].value[2]
                                footPath[ctr,:]=[xf,yf,zf,ar[0],ar[1],ar[2]]
                                ###############################################
                                svPathsMiddle[k][ctr] = x_WS[i][j].value.T
                                pt = x_WS[i][j].value+(ibRho[i][j][k].value*ar.T+SV_HEIGHTS[k]*Z_AXIS).T
                                svPathsLeft[k][ctr] = np.array(pt).flatten()
                                ibRhoR = np.matrix(Ark)*ibRho[i][j].value
                                pt = x_WS[i][j].value+(np.array(ibRhoR[k]).flatten()[0]*ar.T+SV_HEIGHTS[k]*Z_AXIS).T
                                svPathsRight[k][ctr] = np.array(pt).flatten()
                                ctr = ctr+1 
        ctr=0
        for i in range(0,N_walkablesurfaces):
                for j in range(0,len(x_WS[i])):
                        [k,h1,h2,h3] = H
                        thetaV[ctr] = htoq(k,h1,h2,h3)[1]
                        ctr = ctr+1 

        output_folder = os.environ["MPP_PATH"]+"mpp-path-planner/output/homotopy"+str(homotopy)+"/minima"+str(mid)
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        svLeftFname  =   output_folder+"/xpathL.dat"
        svRightFname =   output_folder+"/xpathR.dat"
        svMiddleFname =  output_folder+"/xpathM.dat"
        svQValuesFname = output_folder+"/xpathQ.dat"
        svFootFname = output_folder+"/xpathFoot.dat"

        pickle.dump( svPathsLeft, open( svLeftFname, "wb" ) )
        pickle.dump( svPathsRight, open( svRightFname, "wb" ) )
        pickle.dump( svPathsMiddle, open( svMiddleFname, "wb" ) )
        pickle.dump( thetaV, open( svQValuesFname, "wb" ) )
        pickle.dump( footPath, open( svFootFname, "wb" ) )

if __name__=='__main__':

        homotopy = 0
        mid = 469

        robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"
        ARKname = robot_folder+"/Ark.dat"
        Aleftrightconv = pickle.load( open( ARKname, "rb" ) )
        Ark = Aleftrightconv[mid]

        minimaPath = os.environ["MPP_PATH"]+"mpp-path-planner/output/homotopy"+str(homotopy)+"/minima"+str(mid)
        homotopyPath = os.environ["MPP_PATH"]+"mpp-path-planner/output/homotopy"+str(homotopy)
        x_WS = pickle.load( open( minimaPath+"/x_WS.dat", "rb" ) )
        ibRho = pickle.load( open( minimaPath+"/ibRho.dat", "rb" ) )
        H = pickle.load( open( minimaPath+"/H.dat", "rb" ) )
        pathPlanes = pickle.load( open( homotopyPath+"/pathplanes.dat", "rb" ) )

        upperBodyPathToFile(mid, Ark, x_WS, ibRho, H, pathPlanes, homotopy)
