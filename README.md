#  Dissertation Repository: Stability Guarantees for Model Predictive Controllers 
This repository contains the code developed for the dissertation, titled "Stability Guarantees for Model Predictive Controllers", submitted to obtain the Master of Science Degree in Electrical and Computer Engineering.
The research was supervised by Prof. Daniel de Matos Silvestre and Prof. Rita Maria Mendes de Almeida Correia da Cunha.
The code implements a Model Predictive Control (MPC) approach to generate optimal trajectories for autonomous drones while avoiding obstacles and ensuring safe navigation.

# Abstract
ADICIONAR

## Keywords 
ADICIONAR

# Code organisation
The code in this repository is organised into the following directories and files:

The current directory contains the source code for MPC in 2D (test concept) and 3D (drone test).
  - `MPC2D.m`: Implements MPC with different initial conditions in a Monte Carlo simulation, including set reduction of the region of attraction and obstacle avoidance.
  - `MPC3D.m`: Applies the same methodology to a 3D drone environment. 
  - `ComparisonCZvsCCG.m`: Compares results for the region of attraction using  CZs and CCGs in both 2D and 3D.

  ## CCGFunctions
  This folder contains functions related to CCGs.
   - `boxCCG.m`: Returns the interval that overbounds the CCG.
   - `CCGCartesian.m`: Returns the Cartesian Product between 2 CCGs.
   - `CCGInnerRPI.m`: Returns the inner RPI considering the closed-loop system and the disturbances.
   - `CCGIntersect.m`: From ReachTool by danielmsilvestre. Returns the intersection between 2 CCGs after a linear map.
   - `CCGIntersect2Sets.m`: Adapted from the setOperations by danielmsilvestre. Returns the intersection between 2 CCGs.
   - `CCGLinMap.m`: From ReachTool by danielmsilvestre. Returns the linear map between a matrix and a CCG with the addition of a vector.
   - `CCGMinkowskiSum.m`: From setOperations by danielmsilvestre. Returns the Minkowski sum between 2 CCGs.
   - `CCGOverbound.m`: Returns trivial CCG overbounding simple sets like norm balls.
   - `CCGReduction.m`: Returns the Minkowski Difference of a CCG considering a specific value for reduction.
   - `compileCCG.m`: From ReachTool by danielmsilvestre. Compile the CCG with the tuple information. Returns the constrained set associated with the constrained norm
   - `compileCCGOptimal.m`: Adapted from `compileCCG.m` by danielmsilvestre. Compile the CCG without the explicit tuple information (only with its size). Returns the constrained set associated with the constrained norm.

   ## MPCFunctions
   This folder contains functions related to MPC.
   - `MPCFormulation.m`: Formulates the cost function and constraints for the MPC with a variable terminal set. Returns the cost function, the constraints and the manageable parameters for varying the terminal set.
   - `MPCSimulation.m`: Simulates an mpc trajectory considering a specific initial condition. Returns the controller's trajectory and the diagnostics of each step.

  ## Functions
  This folder contains auxiliary functions that simplify the methodology.
   - `AddZeros.m`: Function created to auxiliate the implementation of an mpc with a variable terminal set. Returns the same CCG but with a higher size due to the zeros added.
   - `BackwardReachabiliySets.m`: Crates the sequence from the RPI to the Region of Attraction considering the system model. Returns the sequence of sets.
   - `FindSet.m`: Considering the initial condition for the mpc looks for the larger set that contains this state. Returns the index from the sequence of sets where the initial conditions belongs.
   - `ForwardReachabiliySets.m`: Considering the procedding applied on the `BackwardReachabiliySets.m` create a new sequence considering the introduction of an obstacle in the middle of the trajectory. Important to note that these newest list is the same until the obstacle appears, then change but always considering that can not be larger then the one previous calculated.
   - `Init2D.m`: Returns the model and the sets needed to initializate the `MPC2D.m`.
   - `InitDrone.m`: Returns the model and the sets needed to initializate the `MPC3D.m`.
   - `PlotGeral.m`: Plot the sequence of sets in 2D and 3D.
   - `Plot2DIn3D.m`: Plot specifically a 3D visualization for a sequence of sets in 2D.
   - `PlotSetObstacles.m`: Plot above the previous visualization of the sequence without the obstacle introduction. It's a good feature to visualize the impact of the obstacle.
   - `PlotSetObstacles3D.m`: Adapted from `PlotSetObstacles.m` for 3D sets.
   - `PlotSetObsIdx.m`: ESCREVER
   - `PlotSets.m`: Plot the constraint sets.
   - `SetReduction.m`: Compute the value that need to be reduced around a set considering the solver's numerical error. Returns the CCG reduced.

# Instructions
## Execution Guidelines

ADICIONAR

## Requirements

ADICIONAR

# Results
The document that has been submitted can be found in the primary directory, and it contains all the presented results.

## MPC Trajectory
The following illustration is a representation of the MPC prediction at each discrete-time instant, followed by the final trajectory.
![](Outputs/mpc_trajectory.gif)