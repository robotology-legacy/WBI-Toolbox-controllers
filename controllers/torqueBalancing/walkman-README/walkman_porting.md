# Adapt iCub balancing infrastructure to robot Walkman
This document is a summary of my work on adapting the iCub torque balancing control to Walkman platform.

### Preliminary operations

- To ensure the installation of all the required dependencies for walkman on my computer, I downloaded the [robotology-superbuild](https://github.com/robotology-playground/robotology-superbuild). To avoid possible conflicts with the iCub dependecies already installed on my computer, and because also ROS was required, I built this repository in a virtual machine. It was then necessary to disable the cmake option `USE_ONLY_OPEN_SOURCE`, and to ask for the access to the private repository `iit_bigman_ros_pkg`. After these preliminary operations, I got both the `bigman.sdf` and the `bigman.urdf` models of Walkman.

- Then, I created a private repository in github named [walkman-gazebo]() (you need permissions to access this link), containing the following .sdf and .urdf models of Walkman:

1) bigman

2) bigman_gazebo

3) bigman_real

4) bigman only legs

5) bigman only upper_body gazebo

6) bigman only upper_body

this folder is, at the current state, in a private repository in git. This because some of the features inside are not public.

### Fixing bugs on the new walkman-gazebo repository

- I am currently working on Ubuntu 16.04 LTS, with Gazebo 7.8.1 and Yarp 2.3.70. When I tried to load the robot models on gazebo from the repository `walkman-gazebo`, I got errors related to the use of some deprecated functions in yarp configuration files (e.g. the file `bigman_gazebo.ini`, and so on). I fixed this bug by eliminating the deprecated functions from all the configuration files. The commit of this fix is [here](https://gitlab.robotology.eu/walkman-drc/iit-bigman-ros-pkg/issues/10) (you need permissions to check this commit). 

### Working on the configuration files for yarpWholeBodyInterface, wholeBodyDynamics3 and torqueBalancing

- torqueBalancing: I created a branch in [WBIToolboxControllers](https://github.com/robotology-playground/WBI-Toolbox-controllers) named `walkman`. Here I added the walkman platform to the list of  available robots. Furthermore, I modified the [codyco-modules](https://github.com/robotology/codyco-modules) repository. In the `torqueBalancing` subdirectory I created a new folder named `bigman`, containing the files:
 
 1) homePoseBalancingTwoFeet.ini: this file contains the home position for walkman joints. This home is coincident with the initial position in Gazebo. Other three similar home positions files have been created.

 2) torqueBalancing.ini and torqueBalancingLeftFoot.ini: it will be necessary to modify all the low-level gains and torque saturation according to the balancing simulation results. 

It was also necessary to modify the `CMakeLists` accordingly. I tested the yarpmotorgui with the robot in gazebo, and it seems everything is working fine.

- yarpWholeBodyInterface: I added the folder `bigman` containing the `yarpWholeBodyInterface.ini` file. I modified this file such that all the joints, FT sensors and IMUs have now the names used in walkman urdf. It was also necessary to modify the CMakeLists accordingly. This folder contains also a copy of the robot urdf.
  
- wholeBodyDynamics3: as before, I created the folder `bigman`. However, I got some difficulties both in the definition of the `wholeBodyDynamics3.ini` and `wholeBodyDynamicsDevice.ini` file and in running the module. In particular, I got the following errors:

 1) No FT sensors found in the urdf model. This was solved by adding to the urdf the following lines:
    
 ```
 <!-- ************ MODIFIED TO FIT THE FT SENSORS
 REQUIREMENTS FOR WHOLEBODYDYNAMICS3 ************ -->
 <!-- <gazebo reference="r_leg_ft_joint"> -->
 	<sensor name="r_leg_ft_sensor" type="force_torque">
		<parent joint="RAnkLat"/>
        	<force_torque>
          		<!-- <frame>sensor</frame> -->
          		<frame>child</frame>
          		<measure_direction>child_to_parent</measure_direction>
        	</force_torque>
                <always_on>1</always_on>
         	<!-- <visualize>1</visualize> -->
          	<update_rate>100</update_rate>
        	<!-- <pose frame=''>0.005 0 -0.107 0 -0 0</pose> -->
        	<!-- <plugin name='RAnkSag_FT_plugin' filename='libgazebo_yarp_forcetorque.so'>
          		<yarpConfigurationFile>model://bigman_urdf/conf/bigman/bigman_gazebo_right_leg_ft.ini</yarpConfigurationFile>
        	</plugin> -->
      	</sensor>
 <!-- </gazebo> -->
<!-- <gazebo reference="l_leg_ft_joint"> -->
 	<sensor name="l_leg_ft_sensor" type="force_torque">
        	<parent joint="LAnkLat"/>
		<force_torque>
          		<!-- <frame>sensor</frame>  -->
          		<frame>child</frame>
          		<measure_direction>child_to_parent</measure_direction>
        	</force_torque>
        	<always_on>1</always_on>  	
         	<!-- <visualize>1</visualize> -->
        	<update_rate>100</update_rate> 	
        	<!-- <pose frame=''>0.005 0 -0.107 0 -0 0</pose>  -->
        	<!-- <plugin name='LAnkSag_FT_plugin' filename='libgazebo_yarp_forcetorque.so'>
          		<yarpConfigurationFile>model://bigman_urdf/conf/bigman/bigman_gazebo_left_leg_ft.ini</yarpConfigurationFile>
        	</plugin> -->
      	</sensor>
<!-- </gazebo> -->
<!-- <gazebo reference="l_arm_ft_joint"> -->
 	<sensor name="l_arm_ft_sensor" type="force_torque">
		<parent joint="LWrj2"/>
        	<force_torque>
          		<!-- <frame>sensor</frame> -->
			<frame>child</frame>
          		<measure_direction>child_to_parent</measure_direction>
        	</force_torque>
        	<always_on>1</always_on> 	
        	<!-- <visualize>1</visualize> -->
        	<update_rate>100</update_rate> 	
        	<!-- <pose frame=''>0 0 -0.0565 0 -0 0</pose> -->
        	<!-- <plugin name='LForearmPlate_FT_plugin' filename='libgazebo_yarp_forcetorque.so'>
          		<yarpConfigurationFile>model://bigman_urdf/conf/bigman/bigman_gazebo_left_arm_ft.ini</yarpConfigurationFile>
        	</plugin> -->
     	 </sensor>
<!-- </gazebo> -->
<!-- <gazebo reference="r_arm_ft_joint"> -->
 	<sensor name="r_arm_ft_sensor" type="force_torque">
		<parent joint="RWrj2"/>
        	<force_torque>
          		<!-- <frame>sensor</frame> -->
			<frame>child</frame>
          		<measure_direction>child_to_parent</measure_direction>
        	</force_torque>
        	<always_on>1</always_on>	
        	<!-- <visualize>1</visualize> -->
        	<update_rate>100</update_rate> 	
        	<!-- <pose frame=''>0 0 -0.0565 0 -0 0</pose> -->
        	<!-- <plugin name='RForearmPlate_FT_plugin' filename='libgazebo_yarp_forcetorque.so'>
          		<yarpConfigurationFile>model://bigman_urdf/conf/bigman/bigman_gazebo_right_arm_ft.ini</yarpConfigurationFile>
        	</plugin> -->
     	 </sensor>
<!-- </gazebo> -->

```
  
 IMPORTANT: since I modified the .urdf of walkman in the `yarpWholeBodyInterface` folder, I decided to rename this file `bigman_torqueBalancing.urdf`. This to avoid to have different models with the same name. 

 2) It was necessary to modify some files in the iDynTree repository. Check the commits [here](https://github.com/robotology/idyntree/commit/2470a71cee49ee3a611195e36b8dc447eca9f154).

### Balancing and YOGA++
The Simulink controller for walkman is fully working for both balancing and left and right demo, and for the Yoga demo. To get all the requirements for working with Walkman in Simulink, follow the instructions [here]() (you need permissions).


