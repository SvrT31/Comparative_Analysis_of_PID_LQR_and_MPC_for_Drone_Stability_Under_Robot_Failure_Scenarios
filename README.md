This repository includes the codes that are involved in running the simulation and drone of my Bachelors Thesis project. This is a work in progress.

## Abstract
One of the most pressing concerns in today’s drone industry is the risk of quadrotor motor failure, particularly in larger drones where such incidents can have severe or even fatal consequences. While mechanical safety measures like parachutes offer some level of protection, they come with limitations such as minimum altitude requirements and potential malfunctions. An alternative and increasingly important approach involves failsafe control strategies that enable the drone to maintain stable flight or execute a controlled landing using its remaining operational motors. Many such failsafe control methods have been proposed, but most remain theoretical and untested, with a noticeable lack of thorough comparative analyses to assess their real-world effectiveness. My project addresses this gap by evaluating the performance of standard control techniques like Proportional-Integral-Derivative (PID) controller, Model Predictive Controller (MPC), and Linear Quadratic Regulator (LQR) under motor failure conditions. A standard DIY Q450 quadcopter drone is used for all the tests so results are easily replicable. This comparative study aims to determine which approach best maintains drone stability in motor failure scenarios.

## Problem Statement 
Many promising techniques exist to address motor failure in quadrotor drones, but there is a critical need for a comprehensive, realistic comparison of PID, LQR, and MPC controllers in conditions that closely mimic real life. Most studies focus on specific control strategies, often relying on mathematical simulations that fall short of capturing real-world complexities like environmental disturbances, sensor noise, and hardware wear. Realistic simulation environments like ROS Gazebo lack the proper simulation environments where such controllers can be tested. Thus, a thorough assessment of these controllers via real-life testing is essential to reveal their robustness and efficiency, ultimately supporting safer, more resilient fault-tolerant control in UAVs.

### Objective of the research 
1. Build a Do-It-Yourself (DIY) drone without a flight controller, primarily based on Raspberry Pi and Arduino to test the effectiveness of various control techniques  in real-life.
2. Test out the effectiveness of the 3 control techniques (PID, MPC, LQR) under single rotor failure configuration for altitude hold.
3. Perform comprehensive analysis on how to avoid accidents after more than one rotor has failed, and give industry deployable valid suggestions.

## Steps to be taken
1. Testing out possible simulation options
2. Building the drone
3. Electronics
4. Drone architecture and testing
5. Base flight
6. Applying PID controller with gain-switching
7. Applying LQR controller
8. Applying MPC controller

