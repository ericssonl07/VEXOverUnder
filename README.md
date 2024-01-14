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

![](https://drive.google.com/uc?id=1nVk1cxvRX8b6U8su8wZ31Krq2hGlJ0Y_)

1. Constructions

**Refer to figure 1.**
$\overline{P_{1}C_{2}P_{6}} \perp \overline{OR_{1}}$
$\overline{P_{1}P_{3}}$ is global east.
$\overline{P_{4}C_{2}P_{7}} \parallel \overline{P_{1}P_{3}}$
$\overline{C_{2}P_{5}} \perp \overline{OC_{2}}$

2. Calculating positional change

**Refer to figure 2.**
Let the wheel radius be $r_{w}$, and $\Delta \theta_{e}$ be the change in encoder position in radians. 
Then $\Delta p = \Delta \theta_{e} \cdot r$.

3. Calculating heading change

* Case 1: Both wheels have the same sign

**Refer to figure 1.**
$\begin{cases} \Delta \theta r_{0} = \Delta L \\ \Delta \theta \left(r_{0}+w\right) = \Delta \theta r_{0} + \Delta \theta w = \Delta \theta R \end{cases}$
$\Rightarrow \Delta \theta w = \Delta R - \Delta L \Rightarrow \Delta \theta = \frac{\Delta R - \Delta L}{w}$

* Case 2: Wheels have the opposite sign

**Refer to figure 3.**
$w = L_{0}R_{0} = OR_{0} - OL_{0} = \frac{\Delta R}{\theta} - \frac{\Delta L}{\theta} = \frac{\Delta R - \Delta L}{\theta} \Rightarrow \Delta \theta = \frac{\Delta R - \Delta L}{w}$

4. Calculating final heading

**Refer to figure 1.**
$\text{m}\angle OC_{2}P_{1} = \frac{\pi}{2} - \Delta \theta$
$\overline{C_{2}P_{4}} \parallel \overline{C_{1}P_{1}} \Rightarrow \text{m}\angle P_{4}C_{2}P_{1} = \theta_{0}$
$\Rightarrow \text{m} \angle OC_{2}P_{4} = \theta_{0} - \left(\frac{\pi}{2} - \Delta \theta \right) = \theta_{0} + \Delta \theta - \frac{\pi}{2}$
$\Rightarrow \text{m} \angle P_{5}C_{2}P_{4} = \frac{\pi}{2} - \left(\theta_{0} + \Delta \theta - \frac{\pi}{2} \right) = \pi - \theta_{0} - \Delta \theta$
$\Rightarrow \text{m} \angle P_{5}C_{2}P_{7} = \pi - \left(\pi - \theta_{0} - \Delta \theta \right) = \theta_{f} = \theta_{0} + \Delta \theta$

5. Calculating straight-line distance

* Case 1: nonzero theta

**Refer to figure 1.**
$r=r_{0} + \frac{w}{2} = \frac{\Delta L}{\Delta \theta} + \frac{w}{2} = OC_{1}=OC_{2}$
Construct the perpendicular bisector of $\overline{C_{1}C_{2}}$, $\overline{OF}$
$C_{1}F=C_{2}F=r\sin{\frac{\Delta \theta}{2}} \Rightarrow C_{1}C_{2} = 2r\sin{\frac{\Delta \theta}{2}}$

* Case 2: zero theta

$C_{1}C_{2} = \Delta L = \Delta R$

6. Change of basis

**Refer to figure 1.**
$\triangle OC_{1}C_{2}$ is isosceles $\Rightarrow \text{m}\angle OC_{1}C_{2} = \frac{\pi - \Delta \theta}{2}$
$\Rightarrow \text{m} \angle C_{2}P_{3}P_{2} = \frac{\pi}{2} - \frac{\pi - \Delta \theta}{2} = \frac{\Delta \theta}{2}$
Define offset $k = \theta_{0} + \frac{\Delta \theta}{2}$
Set up a change of basis $A = \begin{bmatrix} \cos{-k} & \cos{\frac{\pi}{2} - k} \\ \sin{-k} & \sin{\frac{\pi}{2} - k} \end{bmatrix} = \begin{bmatrix} \cos{k} & \sin{k} \\ -\sin{k} & \cos{k} \end{bmatrix}$
In this system, $C_{1}C_{2}$ lies on the horizontal axis and the change in coordinate is merely the displacement calculated in step 5.
To undo the change of basis to get the translation vector in the global coordinate system, multiply by the inverse matrix:
$A^{-1} = \begin{bmatrix} \cos{k} & -\sin{k} \\ \sin{k} & \cos{k} \end{bmatrix}$

7. Summary

* $\Delta \theta = \frac{\Delta R - \Delta L}{w}$
* $d = \begin{cases} 2r\sin{\frac{\Delta \theta}{2}} & \Delta \theta \ne 0 \\ \Delta L & \Delta \theta = 0 \end{cases}$
* $\vec{t} = d \cdot \begin{bmatrix} \cos{k} & -\sin{k} \\ \sin{k} & \cos{k} \end{bmatrix}$

Units for odometry and drive are as follows:
- Rotation: strictly in radians (may consider adding macro `#define deg * π / 180` to enable passing parameters like `180 deg`, becoming $\frac{180\pi}{180} = \pi$ which automatically converts to radians; functions `to_rad` and `to_deg` have been provided if coders do not wish to use macros)
- Distance: flexible (**importantly, ensure that base width and wheel diameter are in the same units as the chosen coordinate/distance unit**)

### Autonomous functions

The autonomous code uses a PID class which abstracts the PID logic to reduce code clutter and improve readability and maintainability.

Autonomous code uses absolute rotations from Odometry instead of periodic headings (0-360 degrees) with a helper function to deal with coterminal angles.
See [Desmos project](https://www.desmos.com/calculator/ycjzeumvpq).
The function $f\left(x, k\right)$ gives the angle closest to $x$ which is coterminal to $k$ (that is, $\exists n \in \mathbb{N}, x = k+2n\pi$). This is useful for autonomous robot motion: consider the case where current heading is $\frac{359\pi}{180}$ and target heading is $\frac{\pi}{180}$. Using heading would make the robot execute a very large reflex turn, which is inefficient. Using rotations with $x=\frac{359\pi}{180}$ and $k=\frac{\pi}{180}$ yields $\frac{361\pi}{180}$ which is a much more efficient turn.

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