#  Dissertation Repository: Stability Guarantees for Model Predictive Controllers 
This repository contains the code developed for dissertation titled "Stability Guarantees for Model Predictive Controllers", submitted to obtain the Master of Science Degree in Electrical and Computer Engineering.
The research was supervised by Prof. Daniel de Matos Silvestre and Prof. Rita Maria Mendes de Almeida Correia da Cunha. 
The code implements a Model Predictive Control (MPC) approach to generate optimal trajectories for autonomous drones while avoiding obstacles and ensuring safe navigation.

# Abstract
Write abstract

## Keywords 
Scorbot ER-7, Serial Manipulator, Image Processing, One line drawing

# Code organization
The code in this repository is organized into the following directories and files:

The current directory contains the source code for MPC in 2d (test concept) and 3d (drone test).
  - `MPC2D.m`: Implements MPC with different initial conditions in a Monte Carlo simulation, including set reduction of the region of attraction and obstacle avoidance.
  - `MPC3D.m`: Applies the same methodology to a 3D drone environment. 
  - `ComparisonCZvsCCG.m`: Compares results for the region of attraction using  CZs and CCGs in both 2D and 3D.

  ## CCGFunctions
  This folder contains functions used related with CCGs.
   - `boxCCG.m`: FALTA ESCREVER
   - `CCGCartesian.m`: Returns the Cartesian Product between 2 CCGs.
   - `CCGInnerRPI.m`: FALTA ESCREVER
   - `CCGIntersect.m`: From the setOperations by danielmsilvestre. Returns the intersection between 2 CCGs after a linear map.
   - `CCGIntersect2Sets.m`: Adapted from the setOperations by danielmsilvestre. Returns the intersection between 2 CCGs.
   - `CCGLinMap.m`: From the setOperations by danielmsilvestre. Returns the linear map between a matrix and a CCG with the addition of a vector.
   - `CCGMinkowskiSum.m`: FALTA ESCREVER
   - `CCGOverbound.m`: FALTA ESCREVER
   - `CCGReduction.m`: Returns the Minkowski Difference of a CCG considering a specific value for reduction.
   - `compileCCG.m`: by danielmsilvestre. Compile the CCG with the tuple information. Returns the constrained set associated with the constrained norm
   - `compileCCGOptimal.m`: Adapted from `compileCCG.m` by danielmsilvestre. Compile the CCG without the explicit tuple information (only with its size). Returns the constrained set associated with the constrained norm.

  ## MPCFunctions
  This folder contains functions used related with MPC.
   - `MPCFormulation.m`: Formulates the cost function and constraints for the MPC with a variable terminal set. Returns the cost function, the constraints and the manageble parameters for varying the terminal set.
   - `MPCSimulation.m`: Simulates an mpc trajectory considering a specific initial condition. Returns the controller's trajectory and the diagnostics of each step.

  ## Functions
  This folder contains auxiliary functions that simplify the methodology.
   - `AddZeros.m`: Function created to auxiliate the implementation of an mpc with variable terminal Set. Returns the same CCG but with an higher size due to the zeros added.
   - `BackwardReachabiliySets.m`: Crates the sequence from the RPI to the Region of Attraction considering the system model. Returns the sequence of sets.
   - `FindSet.m`: Considering the initial condition for the mpc looks for the larger set that contains this state. Returns the index from the sequence of sets where the initial conditions belongs.
   - `ForwardReachabiliySets.m`: Considering the procedding applied on the `BackwardReachabiliySets.m` create a new sequence considering the introduction of an obstacle in the middle of the trajectory. Important to note that these newest list is the same until the obstacle appears, then change but always considering that can not be larger then the one previous calculated.
   - `Init2D.m`: Returns the model and the sets needed to initializate the `MPC2D.m`.
   - `InitDrone.m`: Returns the model and the sets needed to initializate the `MPC3D.m`.
   - `Plot.m`: Plot the sequence of sets in 2D and 3D.
   - `Plot2DIn3D.m`: Plot specifically a 3D visualization for a sequence of sets in 2D.
   - `PlotSetObstacles.m`: Plot above the previous visualization of the sequence without the obstacle introduction. It's a good feature to visualize the impact of the obstacle.
   - `PlotSetObstacles3D.m`: FALTA ESCREVER
   - `PlotSets.m`: FALTA ESCREVER
   - `SetReduction.m`: Compute the value that need to be reduced around a set considering the solver's numerical error. Returns the CCG reduced.


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