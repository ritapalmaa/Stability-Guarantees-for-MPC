#  Dissertation Repository: Stability Guarantees for Model Predictive Controllers 
This repository contains the code developed for the dissertation, titled "Stability Guarantees for Model Predictive Controllers", submitted to obtain the Master of Science Degree in Electrical and Computer Engineering.
The research was supervised by Prof. Daniel de Matos Silvestre and Prof. Rita Maria Mendes de Almeida Correia da Cunha.
The code implements a Model Predictive Control (MPC) approach to generate optimal trajectories for autonomous drones while avoiding obstacles and ensuring safe navigation.

# Abstract
Write abstract

## Keywords 
X, Y, Z

# Code organisation
The code in this repository is organised into the following directories and files:

The current directory contains the source code for MPC in 2D (test concept) and 3D (drone test).
  - `MPC2D.m`: Implements MPC with different initial conditions in a Monte Carlo simulation, including set reduction of the region of attraction and obstacle avoidance.
  - `MPC3D.m`: Applies the same methodology to a 3D drone environment. 
  - `ComparisonCZvsCCG.m`: Compares results for the region of attraction using  CZs and CCGs in both 2D and 3D.

  ## CCGFunctions
  This folder contains functions related to CCGs.
   - `CCGCartesian.m`: Returns the Cartesian Product between 2 CCGs.
   - `CCGIntersect.m`: From the setOperations by danielmsilvestre. Returns the intersection between 2 CCGs after a linear map.
   - `CCGIntersect2Sets.m`: Adapted from the setOperations by danielmsilvestre. Returns the intersection between 2 CCGs.
   - `CCGLinMap.m`: From the setOperations by danielmsilvestre. Returns the linear map between a matrix and a CCG with the addition of a vector.
   - `CCGLinMapWithoutVector.m`: Adapted from the setOperations by danielmsilvestre. Returns the linear map between a matrix and a CCG.
   - `CCGReduction.m`: Returns the Minkowski Difference of a CCG considering a specific value for reduction.
   - `compileCCG.m`: by danielmsilvestre. Compile the CCG with the tuple information. Returns the constrained set associated with the constrained norm
   - `compileCCGOptimal.m`: Adapted from `compileCCG.m` by danielmsilvestre. Compile the CCG without the explicit tuple information (only with its size). Returns the constrained set associated with the constrained norm.

   ## MPCFunctions
   This folder contains functions related to MPC.
    - `MPCFormulation.m`: Formulates the cost function and constraints for the MPC with a variable terminal set. Returns the cost function, the constraints and the manageable parameters for varying the terminal set.
    - `MPCSimulation.m`: Simulates an mpc trajectory considering a specific initial condition. Returns the controller's trajectory and the diagnostics of each step.

  ## Functions
  This folder contains auxiliary functions that simplify the methodology.
   - `AddZeros.m`: Function created to auxiliate the implementation of an mpc with a variable terminal set. Returns the same CCG but with a higher size due to the zeros added.
   - `BackwardReachabiliySets.m`: Crates the sequence from the RPI to the Region of Attraction considering the system model. Returns the sequence of sets.
   - `FindSet.m`: Considering the initial condition for the mpc, look for the larger set that contains this state. Returns the index from the sequence of sets where the initial conditions belong.
   - `ForwardReachabiliySets.m`: Considering the procedure applied to the `BackwardReachabiliySets.m` create a new sequence considering the introduction of an obstacle in the middle of the trajectory. It is important to note that this newest list is the same until the obstacle appears, then changes, always considering that it cannot be larger than the one previously calculated.
   - `Init2D.m`: Returns the model and the sets needed to initialise the `MPC2D.m`.
   - `InitDrone.m`: Returns the model and the sets needed to initialise the `MPC3D.m`.
   - `Plot.m`: Plot the sequence of sets in 2D and 3D.
   - `Plot2DIn3D.m`: Plot specifically a 3D visualisation for a sequence of sets in 2D.
   - `PlotSetObstacles.m`: Plot above the previous visualisation of the sequence without the obstacle introduction. It's a useful feature to visualise the impact of the obstacle.
   - `SetReduction.m`: Compute the value that needs to be reduced around a set considering the solver's numerical error. Returns the CCG reduced.


# Instructions
## Execution Guidelines

Adicionar

## Requirements

Adicionar

# Results
The results are showcased under the directory '/results'. The submitted document can be found in the folder.

## Title 1
The following illustration is a representation of ...
![](outputs/trajectory_planning.gif)

## Title 2
![](outputs/robot_movement.gif)
