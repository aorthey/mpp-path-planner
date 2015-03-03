mpp-path-planner
=======
mpp-path-planner belongs to a set of packages to conduct motion planning for a sliding humanoid robot in arbitrary environments via motion prior informations.
### Description of package
Package uses the prior information from the X space of the robot, plus the stack
of boxes which approximates the workspace, and computes a solution by solving a
set of convex optimization problems
### Dependencies:
 * cvxpy
 * mpp-robot
 * mpp-environment
 * mpp-mathtools

### Experimental results (paper submission IROS 2015)
 * Section III (Footstep optimization)

The results from Figure 5 can be reproduced by
```bash
python foot-space-curve-2homotopies.py
```
Figure 6 can be reproduced by running
```bash
python foot-space-curve-4homotopies.py
```
The averaged times over 10 runs can be reproduced by 
```bash
python foot-space-curve-2homotopies-table.py
python foot-space-curve-4homotopies-table.py
```
The parameters for the experiment are defined in mpp-robot/mpp/robot/robotspecifications.py
