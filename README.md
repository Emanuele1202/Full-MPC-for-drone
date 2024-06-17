# MPC Drone Control Project

## Overview

This project demonstrates the implementation of a Model Predictive Control (MPC) algorithm for a single drone using the acados library in MATLAB. The drone's model is dynamic, and its performance has been tested through experiments conducted in the Gazebo simulation environment. The connection between MATLAB and Gazebo is established using GENOM, a middleware similar to ROS.   

## Usage

### Running the MPC Algorithm

1. Open a terminal and navigate to the `code` directory, then source the environment variables:
   ```bash
   source env.sh
   ```
2. Open Matlab from the terminal:
   ```bash
   matlab
   ```
3. Run the `launch_single_drone.sh` script from the terminal to initialize and launch genom components:
   ```bash
   ./launch_single_drone.sh
   ```
4. Run the gazebo environment `example.world` through the following command:
   ```bash
   gazebo example.world
   ```
5. In Matlab navigate to `single_quadrotor_ocp.m` , which is the main file. There you can modify initial state `x0` and final state `x_end`. 
   A polynomial trajectory will be automatically built.

6. Then you can run the main, the OCP and the feedback are implemented in the `mpc.m` file.

## Experiments
Experiments have been conducted in the Gazebo simulation environment to validate the performance of the MPC algorithm. Below is a snapshot of the GENOM connection and a video of the drone simulation in Gazebo:

[![GENOM Connection](resources/genom_connection.png)](resources/genom_connection.png)

[![MPC Drone Control](resources/single_drone_mpc.gif.mp4)](resources/single_drone_mpc.gif.mp4)

