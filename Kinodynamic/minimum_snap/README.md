# Minimum snap
## Basic principle
This package implements minimum snap to generate a trajectory with fixed nodes and time at each node.
The normal formula is 
![](<img src="https://latex.codecogs.com/svg.image?x=\min_{x}\left\{&space;\frac{1}{2}x^{T}Qx&space;&plus;&space;P^{T}x\right\},&space;s.t.&space;Ax=B,&space;Gx\leq&space;h"/>) 

## Run
By running 'minimum_snap.py', you can get the result of the simulation.
Additionally, some matrices are also saved in directory 'snap_x' and 'snap_y'.

## Simulation
![image](https://github.com/Yang-Yefeng/PathPlanningAlgorithms/blob/main/somefigures/figure/minimum_snap1.png)
![image](https://github.com/Yang-Yefeng/PathPlanningAlgorithms/blob/main/somefigures/figure/minimum_snap2.png)

![image](https://github.com/Yang-Yefeng/PathPlanningAlgorithms/blob/main/somefigures/figure/minimum_snap3.png)
![image](https://github.com/Yang-Yefeng/PathPlanningAlgorithms/blob/main/somefigures/figure/minimum_snap4.png)
