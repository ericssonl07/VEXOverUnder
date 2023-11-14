# VEX Robotics 2023-2024 Game: Over and Under (HKIS Team 936A)

This repository contains code to manipulate the robot movement during the control and autonomous period.

---
---

## Code Features

### Odometry

Odometry is a technique used to determine the changes in a robot's position and orientation by summing up small changes calculated by changes in motor encoder values over time. In this project, odometry is implemented by a tracking function which has its own dedicated thread that calls it frequently. The math assumes that velocity is constant between tracking calls.

936A coders chose to use Odometry over the VEX GPS since the GPS has experienced hard-to-fix tracking issues in the past which reads null values for position and rotation. Odometry is also preferred over the IMU since using displacement to determine position is more accurate than integrating acceleration twice.

The basic concepts for this repository's odometry functions were inspired by the [Purdue Sigbots](https://wiki.purduesigbots.com/software/odometry) website. 936A coders have also done their own calculations and rigorous proofs that the maths hold for all cases that were not addressed in the Sigbots page (such as negative rotation, opposite-direction rotation, etc.).

General sketch of calculation logic:
1. By assuming constant velocity over an interval of time and measuring encoder change, construct two concentric arcs. 
2. Set up and solve a system of equations with two unknowns theta and r.
3. Create a new rotated coordinate system where the straight-line displacement of the robot over the time interval lies on the vertical axis.
4. Calculate the straight-line displacement.
5. Transform the rotated displacement vector back to the global coordinate system to get the global displacement vector.
6. Keep track of the sum of global displacement vectors which gives the robot's position.

Calculations are found below.
It is shown that the math for a positive (counterclockwise) motion holds for negative (clockwise) motion as well as turns where the left and right sides have opposite sign.
![](https://drive.google.com/uc?id=1bYkyDoyf3wlyTf9NkVbIt3CCdYqRhP6D)

Units for odometry and drive are as follows:
- Rotation: strictly in radians (may consider adding macro `#define deg * π / 180` to enable passing parameters like `180 deg` which automatically converts to radians)
- Distance: flexible (**ensure that base width and wheel diameter are in the same units as the chosen coordinate/distance unit**)

### Autonomous functions

The autonomous code uses a PID class which abstracts the PID logic to reduce code clutter and improve readability and maintainability.

Autonomous code uses absolute rotations from Odometry instead of periodic headings (0-360 degrees) with a helper function to deal with coterminal angles.
See [Desmos project](https://www.desmos.com/calculator/ycjzeumvpq).
The function `f` gives the angle closest to `x` which is coterminal to `k`. This is useful for autonomous robot motion: consider the case where current heading is `359π / 180` and target heading is `π/180`. Using heading would make the robot execute a very large reflex turn, which is inefficient. Using rotations with `x=359π / 180` and `k=π / 180` yields `361π / 180` which is a `π / 90` radian turn compared to a `179π / 90` radian turn using heading.

Setting heading calculates the target rotation and uses the PID controller to efficiently execute the turn.
Move first sets the heading to the rotation given by the inverse tangent of the slope of the relative coordinates, then repeatedly updates the target rotation and uses a PID to move efficiently to the target.

### Control functions

---
---

## Github Instructions for 936A Coders

### Pulling
1. Navigate to ~/Documents/vex-vscode-projects/
2. Create and name a new folder.
3. Navigate to terminal.
4. `cd ~/Documents/vex-vscode-projects/{project name here}`
5. `git init`
6. `git remote add origin https://github.com/ericssonl07/VEXOdometry/`
7. `git pull origin main`

### Pushing
1. Navigate to terminal.
2. `cd ~/Documents/vex-vscode-projects/{project name here}`
3. `git add {file or folder}`
4. `git commit -m "{commit message}"`
5. `git push origin main`