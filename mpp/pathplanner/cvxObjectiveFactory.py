from cvxpy import *
import cvxpy as cvx
import numpy as np
import sys
from math import pi
from robot.robotspecifications import *

class CVXObjectiveFactory:
        objfunc = []
        objfuncNames = []

        def __init__(self):
                self.objfunc = 0
                self.objfuncNames = []

        def __str__(self):
                out = ""
                out += "------------------------------\n"
                out += "Objective Factory\n"
                out += "------------------------------\n"
                for i in range(0,len(self.objfuncNames)):
                        out+=self.objfuncNames[i]
                        out+="\n"
                out += "------------------------------\n"
                out += "Total objective functions: "+str(len(self.objfuncNames))+"\n"
                out += "------------------------------\n"
                return out

        def addSmoothnessMinimization(self, W):
                self.objfunc += norm(W)
                self.objfuncNames.append("functional smoothness minimzation")

        def addLineMinimization(self, x_WS, n):
                for i in range(0,len(x_WS)):
                        for j in range(0,len(x_WS[i])-1):
                                gamma = Variable(1,1)
                                self.objfunc += norm(x_WS[i][j]-n*gamma-x_WS[i][j+1])
                self.objfuncNames.append("all points on line minimzation")

        def addInterpointMinimization(self, x_WS):
                for i in range(0,len(x_WS)):
                        for j in range(0,len(x_WS[i])-1):
                                self.objfunc += norm(x_WS[i][j]-x_WS[i][j+1])
                self.objfuncNames.append("interpoint minimzation")

        def getObjectiveFunction(self):
                return self.objfunc
