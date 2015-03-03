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

#### Section IV (Upper-body optimization)

There are three steps involved:
##### (1) Precomputation of the free space inside of the homotopy classes
```bash
cd $MPP_PATH/mpp-environment
python freeSpaceComputationModule.py
```
this should take around 3m20s, and will write the free space representation into a file.
##### (2) Sampling of the simplified irreducible configuration space.
```bash
cd $MPP_PATH/mpp-robot
python main-convex-set-builder.py
```
this should take around 10s and will create N convex sets containing cross-sections of the robot. N depends on the three parameters SAMPLER_H1_STEP,SAMPLER_H2_STEP,SAMPLER_H3_STEP, which should be changed depending on how accurate one wants to sample (they can be found in mpp-robot/src/robotspecifications.py)

##### (3) Computation of N local minima. 
```bash
cd $MPP_PATH/mpp-path-planner
python iros2015/upperbody-space-curves-wall.py
```
Once this computation is done, we have analyzed all local minima of the planning problem. To display one specific minima, you can specify it with the variables FINAL_HOMOTOPY and FINAL_MINIMA in mpp-robot/src/robotspecifications.py.

For the wall experiment, we have choosen FINAL_HOMOTOPY=0 and FINAL_MINIMA=27 to produce the final results. You can reproduce them via
```bash
python iros2015/upperbody-space-curves-final-minima-plot.py
```
The final results will be saved in output/homotopy0/minima27/, where you can find xpathFoot, which contains the foot path, and xpathQ containing the joint values of the robot along the path. 
