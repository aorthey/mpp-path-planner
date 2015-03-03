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
from environment.walkableSurfacesPlotter import *
from environment.scenePlotter import *
from environment.connectorsPlotter import *
from mathtools.linalg import *
from mathtools.walkable import WalkableSurface, WalkableSurfacesFromPolytopes
from robot.htoq import *
from robot.robotspecifications import * 
from pathplanner.connectorsGetMiddlePath import * 
from pathplanner.connectorComputeNormal import * 
from pathplanner.surfaceMiddlePath import * 
from pathplanner.solvePolytopeMinima import * 
from pathplanner.upperBodyPathPlotter import * 
from pathplanner.upperBodyPathToFile import * 

homotopy = FINAL_HOMOTOPY
mid = FINAL_MINIMA

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
upperBodyPathPlotter(mid, homotopy)
