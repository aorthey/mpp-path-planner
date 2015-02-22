import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

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
from environment.walkableSurfacesPlotter import * 
from environment.connectorsPlotter import * 

def upperBodyPathPlotter(mid, hid):
        plot=Plotter()

        minimafolder = os.environ["MPP_PATH"]+"mpp-path-planner/output/homotopy"+str(hid)+"/minima"+str(mid)

        svLeftFname  =   minimafolder+"/xpathL.dat"
        svRightFname =   minimafolder+"/xpathR.dat"
        svMiddleFname =  minimafolder+"/xpathM.dat"
        svQValuesFname = minimafolder+"/xpathQ.dat"
        svFootFname =    minimafolder+"/xpathFoot.dat"

        svPathsLeft  = pickle.load( open( svLeftFname, "rb" ) )
        svPathsRight = pickle.load( open( svRightFname, "rb" ) )
        svPathsMiddle = pickle.load( open( svMiddleFname, "rb" ) )
        thetaV = pickle.load( open( svQValuesFname, "rb" ) )
        footPath= pickle.load( open( svFootFname, "rb" ) )

        env_folder = os.environ["MPP_PATH"]+"mpp-environment/output"
        Wsurfaces= pickle.load( open( env_folder+"/wsurfaces.dat", "rb" ) )
        Connector_Vstack= pickle.load( open( env_folder+"/connector_vstack.dat", "rb" ) )

        connectorsPlotter(plot,Connector_Vstack)
        walkableSurfacesPlotter2(plot,Wsurfaces)

        plot.lines(svPathsLeft,style='-or',colorIn=COLOR_SWEPTVOLUME_LEFT, \
                        zorder=ZORDER_SWEPTVOLUME)
        plot.lines(svPathsRight,style='-om',colorIn=COLOR_SWEPTVOLUME_RIGHT, \
                        zorder=ZORDER_SWEPTVOLUME)
        #plot.lines(svPathsMiddle,style='-ok')
        #plot.ax.axis('equal')
        #plot.set_view(151,42)
        plot.set_view(57,39)
        plot.showEnvironment()


if __name__ == '__main__':

        minima = 19
        homotopy = 0
        upperBodyPathPlotter(minima, homotopy)

