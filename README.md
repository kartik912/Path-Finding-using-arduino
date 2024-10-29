# Path-Finding-using-arduino

## Tech Stack
Language: C++
Platform: Arduino/Embedded Systems (Pololu 3pi+ 32U4 robot)
Libraries: Pololu Libraries (e.g., Pololu3piPlus32U4 for hardware control, PololuMenu for display interaction, Pololu3piPlus32U4IMU for the inertial measurement unit (IMU))
## Project Summary
This project is a maze-solving robot designed to autonomously navigate and solve mazes using a simplified version of PID (Proportional-Integral-Derivative) control for line following and pathfinding logic. Here are the main points:

Autonomous Maze Solving: Programmed the robot to detect intersections and choose optimal paths through a maze using line sensors, with a modular design for path simplification and optimal route following.

Path Optimization Algorithm: Implemented a path simplification method to condense a sequence of turns, reducing unnecessary steps and improving efficiency, while managing a limited path buffer.

Hardware Integration: Utilized a Pololu 3pi+ robot's components (motors, sensors, IMU, and buttons) with Pololu libraries for navigation control, user input, and real-time feedback via buzzer and display.

Robot used : https://www.pololu.com/product/3736
