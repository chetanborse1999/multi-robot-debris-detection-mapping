# Multi Robot Systems Final Project: Debris Detection using Swarm Robots
This page briefly describes the implementation of multi-agent distributed debris detection and mapping algorithm in MATLAB. This is in partial fulfillment of grade requirement for Multi-Robot Systems (MAE 598). Refer to report pdf for mathematical proofs and implementation details.

## Objective

There are debris fields floating across the ocean surface possibly from a aircraft crash, shipwreck, or even garbage patch/trash vortex. Their positions are difficult to predict due to weather and ocean currents. We aimed to create a Multi-Robot system that traverses the domain, detects the target, maps it and shares the map with other identical agents resulting in convergence to a single unified map. The idea is to use a satellite image as a starting reference and sufficiently explore the domain to achieve maximum coverage.

## Simulation

The simulation is carried out in 2D plane 50x50 continuous grid. The Robot positions, are initialized randomly and so are the actual debris positions. Using a reference satellite image a artificial potential field is created (gaussian) which will act as reference for initial navigation. The robots follows the gradient of the potential functions to reach the estimated position. 
<p align="center"><img src="https://raw.githubusercontent.com/chetanborse1999/multi-robot-debris-detection-mapping/refs/heads/main/potential_function.png" width="640"></p>
<!---
![potential_function](https://raw.githubusercontent.com/chetanborse1999/multi-robot-debris-detection-mapping/refs/heads/main/potential_function.png)
-->
If the debris field is detected within its detection radius, it is mapped to its local occupancy grid. If not, the robots will continue to follow the gradient until a local maximum is reached. Then, it switches to exploration mode and performs a random walk to leave the maximum of potential field and resumes following the gradient after a few timesteps.
<p align="center"><img src="https://github.com/chetanborse1999/multi-robot-debris-detection-mapping/blob/main/trajectories.gif?raw=true" width="640"></p>

Mathematically, after sufficient number of timesteps the individual maps/occupancy grids converge. In some cases, due to nature of exploration, some parts of domain may go unexplored and might miss a few points but generalizes well enough to randomly initialized world. 
<p align="center"><img src="https://raw.githubusercontent.com/chetanborse1999/multi-robot-debris-detection-mapping/refs/heads/main/occupancy_grid_1_2.png" width="640"></p>

The estimated position is shown by Red Cross, Actual position by Blue Cross and Robots by Blue Circle, and trajectory of each robot in a different color. As observed, the Robots do not waste time exploring areas where the debris is less likely to be found and does a fairly good job of exploring the region of high likelihood of detection.
<p align="center"><img src="https://github.com/chetanborse1999/multi-robot-debris-detection-mapping/blob/main/trajectories.png?raw=true" width="640"></p>
