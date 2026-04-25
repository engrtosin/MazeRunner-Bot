# Maze Robot

This repository contains code for a robot that can navigate an unknown (fully connected) maze and collect and rescue balls.

## Structure

- `MazeRunner/`  
  Contains the main robot code used for navigation, wall following, and task execution.

- `dev_utils/`  
  Contains supporting code used during development to implement, test, and validate individual functionalities (e.g., sensors, motor control, and algorithms).

## Notes

The `MazeRunner` folder represents the integrated system, while `dev_utils` includes modular experiments and test implementations that supported development.

### Connection Notes
Wheel Motors
Left
White - M1B, Red - M1A
Right
White - M2A, Red - M2B
In this config, both motors run forward.

Roller Motors
IN1 -> GND, IN2 -> pin5, OUT1 -> Motor Red, OUT2 -> Motor Black

REMINDER: If the robot gets stuck, use the encoder to determine if the robot is stuck and tell it to back up.