mpp-path-planner
=======
mpp-path-planner belongs to a set of packages to conduct motion planning for a sliding humanoid robot in arbitrary environments via motion prior informations.
### Description of package
Package uses the prior information from the X space of the robot, plus the stack
of boxes which approximates the workspace, and computes a solution by solving a
set of convex optimization problems
### Dependencies
 * cvxpy
 * scipy
 * networkx
 * mpp-robot
 * mpp-environment
 * mpp-mathtools

### Install
```bash
mkdir -p ~/devel/mpp/
cd ~/devel/mpp
git clone git@github.com:orthez/mpp-robot.git
git clone git@github.com:orthez/mpp-environment.git
git clone git@github.com:orthez/mpp-mathtools.git
git clone git@github.com:orthez/mpp-path-planner.git
export MPP_PATH="/home/`whoami`/devel/mpp/"
echo 'export MPP_PATH="/home/`whoami`/devel/mpp/"' >> ~/.bashrc
cd $MPP_PATH/mpp-path-planner
```
or in a one-liner:
```bash
mkdir -p ~/devel/mpp/ && cd ~/devel/mpp && git clone git@github.com:orthez/mpp-robot.git && git clone git@github.com:orthez/mpp-environment.git && git clone git@github.com:orthez/mpp-mathtools.git && git clone git@github.com:orthez/mpp-path-planner.git && export MPP_PATH="/home/`whoami`/devel/mpp/" && echo 'export MPP_PATH="/home/`whoami`/devel/mpp/"' >> ~/.bashrc && cd $MPP_PATH/mpp-path-planner
```
### Experimental results (paper submission IROS 2015)
#### Section III (Footstep optimization)

The results from Figure 5 can be reproduced by
```bash
python iros2015/foot-space-curve-2homotopies.py
```
Figure 6 can be reproduced by running
```bash
python iros2015/foot-space-curve-4homotopies.py
```
The averaged times over 10 runs can be reproduced by 
```bash
python iros2015/foot-space-curve-2homotopies-table.py
python iros2015/foot-space-curve-4homotopies-table.py
```
The parameters for the experiment are defined in mpp-robot/mpp/robot/robotspecifications.py
