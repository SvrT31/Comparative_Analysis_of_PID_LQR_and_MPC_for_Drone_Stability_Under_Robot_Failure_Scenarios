This repository includes the codes that are involved in running the simulation and drone of my research project. This is a work in progress. Google docx report can be found [here](https://docs.google.com/document/d/1sgWU8yRvNyqL0LPePdABMbTXuWoa2HvK0gUAs3excbk/edit?usp=sharing).

The program "manual_tuning.py" is used to find the hold position for singular motors. 

"lqr_controller.py" and "mpc_controller.py" are just shells of how the LQR and MPC controllers are to be implemented. Thus, they are currently incomplete. 

"pid_control.py" code is the current working implementation of the PID controler. 

"adagrad_tuning.py" is the adaptation of gradient descent for PID tuning. The source article on how to do this is linked [here](https://towardsdatascience.com/pid-controller-optimization-a-gradient-descent-approach-58876e14eef2).

The "arduino" directory just contains the code for arduino to take data from the controller and send it to the escs which are connected to the motors.

## Abstract
One of the most pressing concerns in today’s drone industry is the risk of quadrotor motor failure, particularly in larger drones where such incidents can have severe or even fatal consequences. While mechanical safety measures like parachutes offer some level of protection, they come with limitations such as minimum altitude requirements and potential malfunctions. An alternative and increasingly important approach involves failsafe control strategies that enable the drone to maintain stable flight or execute a controlled landing using its remaining operational motors. Many such failsafe control methods have been proposed, but most remain theoretical and untested, with a noticeable lack of thorough comparative analyses to assess their real-world effectiveness. My project addresses this gap by evaluating the performance of standard control techniques like Proportional-Integral-Derivative (PID) controller, Model Predictive Controller (MPC), and Linear Quadratic Regulator (LQR) under motor failure conditions. A standard DIY Q450 quadcopter drone is used for all the tests so results are easily replicable. This comparative study aims to determine which approach best maintains drone stability in motor failure scenarios.

## Problem Statement 
Many promising techniques exist to address motor failure in quadrotor drones, but there is a critical need for a comprehensive, realistic comparison of PID, LQR, and MPC controllers in conditions that closely mimic real life. Most studies focus on specific control strategies, often relying on mathematical simulations that fall short of capturing real-world complexities like environmental disturbances, sensor noise, and hardware wear. Realistic simulation environments like ROS Gazebo lack the proper simulation environments where such controllers can be tested. Thus, a thorough assessment of these controllers via real-life testing is essential to reveal their robustness and efficiency, ultimately supporting safer, more resilient fault-tolerant control in UAVs.

### Objectives of the research 
1. Build a Do-It-Yourself (DIY) drone without a flight controller, primarily based on Raspberry Pi and Arduino to test the effectiveness of various control techniques  in real-life.
2. Test out the effectiveness of the 3 control techniques (PID, MPC, LQR) under single rotor failure configuration for altitude hold.
3. Perform comprehensive analysis on how to avoid accidents after more than one rotor has failed, and give industry deployable valid suggestions.

## Steps to be taken
1. Testing out possible simulation options
2. Building the drone
3. Electronics
4. Drone architecture and testing
5. Base flight
6. Applying PID controller
7. Applying gain-switching for PID control
8. Applying LQR controller
9. Applying MPC controller

## Simulation

In order to compare the given control techniques, I had to first use a simulation environment and verify the usability of my controllers. This was because when testing failure controllers, the possibility of the drone getting destroyed is also rather high. In order to do this, I explored  various simulation environments.

My first approach was to use a SITL like PX4 SITL or ArduCopter SITL. This would be ideal if Pixhawk controllers allowed users to give RPMs to individual motors. After spending about 1 month in PX4 documentation and SITL files, I found out that there were rostopics which give thrust to the motors, but ever since the 2018 release, end users were not allowed to access them. Since my laptop hardware does not have the capacity to run Ubuntu 18.04 (as it has run out of LTS support), I thought about developing my own SITL.

I was partly successful in it, as I simulated a proper Typhoon H480 drone in the Ignition Garden version of the Gazebo simulator. I included an Inertial Measurement Unit (IMU) and a Pressure sensor for altitude detection. But I quickly found out that there was no plugin to generate thrust via motor commands, and I would have to code it from scratch. As I am not familiar with coding Gazebo plugins, it would take me about a month to add these plugins to it.

Due to a lack of time, it was decided to not persue this. If anyone wants to simulate a drone in gazebo with the ability to control rpms of individual motors, they would have to start from writing the plugins.

## Drone Building

I first approached the ADAPT Lab (Drone Lab) at VIT Chennai, but soon found out that Pixhawk (which is the flight controller on all the drones in the lab) does not allow base level control on its real pixhawk controller at all. ADAPT Lab did not have a drone with Raspi and Arduino combination which I needed. Thus, I had to build my own drone and deal with all 3 aspects of robotics on my own. Mechanics, Electrical, and Programming. Programming was dealt with in the simulation itself, thus, electrical and mechanical aspects were left.

### Mechanics

I started with the mechanical aspects first as it would give me proper understanding of weight, and thus, what kind of motors and batteries I should use. 

In order to have a universal drone, I had to buy one with 45 degree arms where all motors were upright. Thus, I chose the Q450 drone chassis.

Landing gear was added to this as I planned on adding the battery at the bottom. I also drilled 4 holes to insert zip ties in it for the battery. This finished majority of the physical body of the drone


### Eletronics

For the electrical aspects, the most important component were the motors. In order to lift the drone up, I needed 1000kV A2122 BLDC motors and 2 sets of 1045 propellers. 

For the power source, I went with a 3300mAh 3S 35C battery , which seemed enough to keep my drone afloat for about 20 minutes. These batteries require special chargers. So I bought the cheapest effective battery charger. The imaxRC B3 Compact Charger.

Next was the processor which would handle the control system itself. I first started with a basic Arduino Uno, but realised that Arduinos (no version of them) are capable of processing advanced control techniques like MPC and LQR controllers. Both of them are computationally taxing and need a dedicated processing unit. Thus, I got a Raspberry Pi 4B with 8GB RAM. I already had a 64 GB Sandisk SD card which served as its memory chip. Since Arduino is already known to be great at controlling motors and since I already had one laying around, I used it.

I also needed sensors which could give me the orientation of the drone and the altitude. For this, I chose a BMP280 altitude sensor and a MPU9250 IMU. I later found out that I could have just purchased a GY91 sensor which combined these 2 sensors in one, but it was too late. These websites [(Raspi to Arduino)](https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Conclusion), [(MPU9250 to Raspi)](https://www.instructables.com/How-to-Connect-MPU9250-and-Raspberry-Pi-Part-1/), and [(BMP280)](https://iotstarters.com/configuring-bmp280-sensor-with-raspberry-pi/#:~:text=Connecting%20BMP280%20sensor%20with%20Raspberry%20Pi,-In%20this%20project&text=The%20BMP280%20module%20has%206,of%20the%20Raspberry%20Pi%20board) gave proper guidance on how to connect all this together. 

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

## Future Steps
1. Finding Kp, Ki, and Kd for when all 4 motors are active and developing a rotor 4 kill switch and simultaneous gain-switching.
2. Apply LQR controller with both trirotor and quadrotor control and switch mechanism.
3. Apply MPC controller with both trirotor and quadrotor control and switch mechanisms.
4. Compare performance 

The base code for LQR and MPC controllers is already written down. The logic was taken from [this](https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/) and [this](https://www.instructables.com/Quadcopter-MPC-Control/) website.

## Results 
Currently, the first 5 out of 8 steps have been completed. The plans for simulation testing have been discarded. PID testing has shown promising results and the drone is ready to be used with the real controllers.

The google docs report will be updated as soon as possible, and final code will see changes once the testing is complete.

Please feel free to contact me at my email address - tembesuvrat@gmail.com



