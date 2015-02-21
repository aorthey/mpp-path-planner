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

