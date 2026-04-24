# Maze Robot

This repository contains code for a robot that can navigate an unknown (fully connected) maze and collect and rescue balls.

## Structure

- `MazeRunner/`  
  Contains the main robot code used for navigation, wall following, and task execution.

- `dev_utils/`  
  Contains supporting code used during development to implement, test, and validate individual functionalities (e.g., sensors, motor control, and algorithms).

## Notes

The `MazeRunner` folder represents the integrated system, while `dev_utils` includes modular experiments and test implementations that supported development.

Left Motor
White - M1B, Red - M1A
Right Motor
White - M2A, Red - M2B
In this config, both motors run forward.