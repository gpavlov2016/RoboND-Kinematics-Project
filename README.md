# Robotic arm - Pick & Place project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/theta1.png
[image3]: ./misc_images/kuka-side.png
[image4]: ./misc_images/misc2.png


### Kinematic Analysis
#### 1. The kinematics analysis was performed with the help of Rviz by attaching a reference to each frame according to DH convention

![alt text][image1]

#### 2. Here are the resulting DH parameters derived from the kinematics analysis

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

The homogeneous transform between subsequent links (i=1..7)were generated using the following matrix structure as defined by the DH method:
```
T(i-1)_i = Matrix([[               cos(qi),                -sin(qi),                0,                  a(i-1)],
                  [sin(qi)*cos(alpha(i-1), cos(qi)*cos(alpha(i-1)), -sin(alpha(i-1)),     -sin(alpha(i-1))*d1],
                  [sin(qi)*sin(alpha(i-1), cos(qi)*sin(alpha(i-1)),  cos(alpha(i-1)),      cos(alpha(i-1))*d1],
                  [                     0,                       0,                0,                      1]])
```
After the individual HTs were calculated the base_link-->gripper combined HT was calculated by multiplying the subsequent HT matrices:
```
T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
```

#### 3. Decoupling of Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics

KR210 robot has spherical wrist (the last 3 revolute joints axes intersect in a single point) therefore the IK problem can be decoupled to Position problem and Orientation problem. For the inverse position problem the first step is to calculate the wrist center (joint4) by subtracting  the rotated length of link 6 from the gripper required position:
```
nx = Rot_G[0,2]
ny = Rot_G[1,2]
nz = Rot_G[2,2]
d = 0.303
# WC - Wrist Center
wx = px - d*nx
wy = py - d*ny
wz = pz - d*nz

```
Given the required position of the wrist center, the first three angles can be calculated using trigonometry starting with theta1 by projecting the wrist center to x-y plane as follows:

![alt text][image2]

For theta2 and theta3 we use a triangle formed by J2, J3 and J5(WC). Following image visualizes the angle and link definitions:

![alt text][image3]

The last three are theta4, theta4 and theta6 and those angles affect only the orientation of the gripper. Here the HTs we calculated previously are coming handy. We first find the rotation matrix from frame3 to frame6:
```
R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
R3_6 = R0_3.inv("LU") * Rot_G
```
Where:  
R0_3  - Rotation matrix from frame0 to frame3  
T0_1  - Homogenous Transform matrix from frame0 to frame1  
Rot_G - Rotation matrix from frame0 to gripper (frame6)  
R3_6  - Rotation matrix from frame3 to frame6  

Now we can extract the last three theta values using Euler angles using the method described here: http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf
```
x = R3_6[0,2]
y = R3_6[1,2]
z = R3_6[2,2]
theta4 = atan2(z, -x)
theta5 = atan2(sqrt(x**2 + z**2), y)
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Optimization
Some of the inverse kinematics operations take considerable amount of compute resources therefore most of the matrices that do not change during execution, such as the T0_1, T1_2 etc..., are precomputed outside of `handle_calculate_IK` function so they run only once when the server is started, saving around 0.8sec per trajectory. 

### And finally the robot in action
![alt text][image4]

