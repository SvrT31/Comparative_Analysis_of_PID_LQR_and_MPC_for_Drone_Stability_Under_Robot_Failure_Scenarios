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
4. Program architecture and testing
5. Base flight
6. Applying PID controller with gain-switching
7. Applying LQR controller
8. Applying MPC controller

## Simulation

In order to compare the given control techniques, I had to first use a simulation environment and verify the usability of my controllers. This was because when testing failure controllers, the possibility of the drone getting destroyed is also rather high. In order to do this, I explored  various simulation environments.

My first approach was to use a SITL like PX4 SITL or ArduCopter SITL. This would be ideal if Pixhawk controllers allowed users to give RPMs to individual motors. After spending about 1 month in PX4 documentation and SITL files, I found out that there were rostopics which give thrust to the motors, but ever since the 2018 release, end users were not allowed to access them. Since my laptop hardware does not have the capacity to run Ubuntu 18.04 (as it has run out of LTS support), I thought about developing my own SITL.

I was partly successful in it, as I simulated a proper Typhoon H480 drone in the Ignition Garden version of the Gazebo simulator. I included an Inertial Measurement Unit (IMU) and a Pressure sensor for altitude detection. But I quickly found out that there was no plugin to generate thrust via motor commands, and I would have to code it from scratch. As I am not familiar with coding Gazebo plugins, it would take me about a month to add these plugins to it.

Due to a lack of time, it was decided to not persue this. If anyone wants to simulate a drone in gazebo, they would have to start from writing the plugins.

## Drone Building

I first approached the ADAPT Lab (Drone Lab) at VIT Chennai, but soon found out that Pixhawk (which is the flight controller on all the drones in the lab) does not allow base level control on its real pixhawk controller at all. ADAPT Lab did not have a drone with Raspi and Arduino combination which I needed. Thus, I had to build my own drone and deal with all 3 aspects of robotics on my own. Mechanics, Electrical, and Programming. Programming was dealt with in the simulation itself, thus, electrical and mechanical aspects were left.

####Mechanics

I started with the mechanical aspects. In order to have a universal drone, I had to buy one with 45 degree arms where all motors were upright. Thus, I chose the Q450 drone chassis.
Landing gear was added to this as I planned on adding the battery at the bottom. I also drilled 4 holes to insert zip ties in it for the battery. This finished majority of the physical body of the drone

### Eletronics

For the electrical aspects, the most important component were the motors. In order to lift the drone up, I needed 1000kV A2122 BLDC motors and 2 sets of 1045 propellers. 

For the power source, I went with a 3300mAh 3S 35C battery , which seemed enough to keep my drone afloat for about 20 minutes. These batteries require special chargers. So I bought the cheapest effective battery charger. The imaxRC B3 Compact Charger.

Next was the processor which would handle the control system itself. I first started with a basic Arduino Uno, but realised that Arduinos (no version of them) are capable of processing advanced control techniques like MPC and LQR controllers. Both of them are computationally taxing and need a dedicated processing unit. Thus, I got a Raspberry Pi 4B with 8GB RAM. I already had a 64 GB Sandisk SD card which served as its memory chip. Since Arduino is already known to be great at controlling motors and since I already had one laying around, I used it.

I also needed sensors which could give me the orientation of the drone and the altitude. For this, I chose a BMP280 altitude sensor and a MPU9250 IMU. I later found out that I could have just purchased a GY91 sensor which combined these 2 sensors in one, but it was too late. Websites [25-27] gave proper guidance on how to connect all this together.

To power all of this, I had to get a power distribution board (PDB). This is because Arduino can only take up to 12A of power and Raspberry Pi can only deal with 5A. Any more, and they would shut down. Thus, I bought a XT60 Power Distribution Board. I also needed Electronic Speed Controllers (ESCs) for the motors, and since Raspberry Pi and Arduino use specific input voltages, I had to acquire and solder them onto the board. Thus, I bought a XT60 Power Distribution Board and soldered the EScs on it. I added an arduino power pin and a C-type wire for powering Arduino and Raspberry Pi.

The electrical connections of the drone look as follows -
![Screenshot from 2024-11-11 19-27-30](https://github.com/user-attachments/assets/5676de87-a953-4ad2-ba30-3cd061b4e44c)

A mechanical damper had to be added to the drone in order to reduce the noise coming from MPU9250’s accelerometer. The final drone looked like this -
![WhatsApp Image 2024-11-12 at 11 14 46 PM](https://github.com/user-attachments/assets/cad437a1-e069-4e34-b0e4-bc05aca4cc13)

## Program Architecture and Testing

The flow of the programs looks like this -
![Screenshot from 2024-11-11 19-14-25](https://github.com/user-attachments/assets/1aedcfc5-69ae-488f-b058-b9f7a3ee4b36)

For LQR and MPC controllers, the PID loops are changed with LQR and MPC control loops respectively.

The final part is real-life implementation of the control techniques for stability hold. 

For this, I took the raw data coming from the pressure sensor in hPa and converted it to metres using the equation-
![image](https://github.com/user-attachments/assets/121c6b3c-3354-4ae2-a35d-3788d65bd4bc)

Where,
Pressure is the data coming in hPa from the sensor,
and Pressure at Sea Level is 1013.25 hPa

For the IMU data, I received raw readings of Acceleration in the X-, Y- and Z-axis. I converted them to Roll and Pitch using these equations -
![image](https://github.com/user-attachments/assets/cb7bd7da-3831-4ca6-9981-2833753917f5)

The real life implementation created problems such as sensor noise, overheating in Raspberry Pi and serial communication delays. In order to solve the IMU sensor noise, I set the IMU at the lowest frequency and added an Exponential Moving Average filter. This reduced the noise to an acceptable level. Heatsinks were added to Raspi and serial buffer auto cleaning was added to the code.

At first, holding the altitude was not the primary concern. The orientation and stability of the drone was prioritised.  Drone testing was done in front of the Automotive Garage at VIT Chennai building by using the Ziegler-Nichols PID tuning method. In order to hold the drone at a certain height, the motors are tuned to add or subtract values from a base value of 66. While landing or launching, the drone first meets these values and then goes up or down according to commands. 

Base testing involves testing of individual control systems and all the motors. The drone testing was done in the Automotive garage. Here, it was found that the drone holds stability at the exact pwm value of 66. Also, with really low PID settings, the drone is also able to hold its weight and stop from crashing down.
Video -
https://github.com/user-attachments/assets/7469a454-adf2-4a38-8daa-7c078af32e0d




