# CSCI6555_COMPUTER_ANIMATION_FOURTH_LAB-Public

Behavioral Motion Control System
CS 6555 – Computer Animation
Assignment 4
1. Overview
This project implements a 3D behavioral motion control system based on Reynolds’ flocking model using OpenGL and GLUT.
The system consists of:
One leader boid controlled by keyframed motion
Multiple follower boids that react to the leader and to each other using behavioral rules
The output is a real-time animated simulation of flocking behavior in 3D space.
2. Platform and Environment
Operating System: macOS
Graphics API: OpenGL (Apple framework)
Windowing Toolkit: GLUT
Language: C++
Note: OpenGL is deprecated on macOS but fully functional for this assignment.
3. Compilation and Execution (macOS)
Compile
g++ lab4.cpp -o lab4 \
-framework OpenGL \
-framework GLUT
Run
./lab4
4. System Architecture
4.1 Leader Boid (Global Control)
The leader boid follows a keyframed trajectory defined by control points containing:
Euler angles (rotation)
3D position
The leader motion is computed by:
Converting Euler angles to quaternions
Interpolating orientation and position using a spline-based interpolation
Converting the quaternion to a 4×4 transformation matrix
Applying the matrix in OpenGL to animate the leader smoothly
This provides global control over the flock.
4.2 Follower Boids (Behavioral Control)
Each follower boid updates its motion every frame using four behavioral rules:
1. Leader Following
Boids are gently attracted toward the leader position.
2. Collision Avoidance (Separation)
Boids repel each other when they come within a short distance to prevent crowding.
3. Velocity Matching (Alignment)
Boids attempt to match their velocity with the average velocity of nearby boids.
4. Flock Centering (Cohesion)
Boids are pulled toward the center of the flock.
4.3 Behavior Mediation
The final velocity of each boid is computed using a weighted sum of all behavior vectors.
Each behavior is scaled by a constant factor to ensure smooth and stable motion.
5. Rendering and Animation
Boids are rendered as 3D spheres
The leader boid is larger than follower boids
Lighting, materials, and depth testing are enabled
The camera is positioned using gluLookAt
Animation runs at approximately 60 frames per second
6. Input and Output
Input
Initial boid positions
Keyframe control points for the leader
Behavior parameters (weights and thresholds)
Output
A real-time animated flocking simulation in 3D
7. Extensions Implemented
Keyframed global control for leader motion
Quaternion-based orientation interpolation
Full 3D flocking system
8. Possible Future Extensions
Banking boids based on velocity and curvature
Environmental obstacles
Predator-prey behaviors
Learning-based behavior control (GA / ML)
9. Files Included
lab4.cpp – Source code for the behavioral motion control system
README.md – Project documentation
10. References
Reynolds, C. W. Flocks, Herds, and Schools: A Distributed Behavioral Model, SIGGRAPH 1987
