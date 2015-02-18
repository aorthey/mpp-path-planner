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
from mathtools.timer import *
from mathtools.walkable import WalkableSurface, WalkableSurfacesFromPolytopes
from robot.htoq import *
from robot.robotspecifications import * 
from pathplanner.connectorsGetMiddlePath import * 
from pathplanner.connectorComputeNormal import * 
from pathplanner.surfaceMiddlePath import * 
from environment.urdfparser import URDFtoPolytopes


def footstepPathPlotter(env_fname, K = inf):
        env_folder = os.environ["MPP_PATH"]+"mpp-environment/output"
        Wsurfaces = pickle.load( open( env_folder+"/wsurfaces.dat", "rb" ) )

        pobjects = URDFtoPolytopes(env_fname)
        wplot=Plotter()
        wplot.allPolytopes(pobjects)
        wplot.allWalkableSurfaces(Wsurfaces)

        ctr=0
        fname = env_folder+"/footsteppath"+str(ctr)+".dat"
        colors="kgmb"
        while os.path.exists(fname) and ctr<K:
                print "Plotting path in homotopy class",ctr
                pts=pickle.load( open( fname, "rb") )
                if len(pts)>0:
                        X=pts[:,0]
                        Y=pts[:,1]
                        Z=pts[:,2]
                        X=np.squeeze(X)
                        Y=np.squeeze(Y)
                        Z=np.squeeze(Z)
                        #wplot.ax.scatter(X,Y,Z,marker='o',c='r',s=5)
                        wplot.ax.plot3D(X,Y,Z,'-ok',color=colors[ctr%len(colors)],linewidth=3,markersize=5,zorder=100)

                ctr+=1
                fname = env_folder+"/footsteppath"+str(ctr)+".dat"


        wplot.set_view(-90,90)
        #wplot.ax.set_aspect('equal', 'datalim')
        #wplot.ax.set_xlim(-4, 4)
        #wplot.ax.set_ylim(0, 10)
        #wplot.ax.set_zlim(0, 3)
        #wplot.point([0,0,2.1],color=COLOR_START_POINT,size=200,zorder=10)
        #wplot.point([0,8,2.1],color=COLOR_START_POINT,size=200,zorder=10)

        wplot.show()

if __name__ == "__main__":
        #env_fname = os.environ["MPP_PATH"]+"mpp-environment/urdf/staircase_stones.urdf"
        #env_fname = os.environ["MPP_PATH"]+"mpp-environment/urdf/quatro_homotopy.urdf"
        env_fname = os.environ["MPP_PATH"]+"mpp-environment/urdf/wall.urdf"
        footstepPathPlotter(env_fname,1)
