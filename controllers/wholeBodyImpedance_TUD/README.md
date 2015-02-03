
This simulink model implements inverse dynamics + impedance control on the whole body.
The inverse dynamics controller aims at making the iCub joints following a trajectory. As no balance strategies is implemented, the iCub is expected to be fixed to the root.

=== 

In Gazebo, the iCub (fixed) has to be used.
Moreover, the flag icub_fixed has to be set to true in the icubGazeboSim.ini file. This file can be found in: 

.../codyco-superbuild/build/install/share/codyco/context/wbit/

===

In the Simulink model, you can notice that Gaussian noise blocks are used to take into account the noise in both sensors and actuators. Also, a perturbation block has been added to simulate the external torques applied on the robot. Regarding the data, two small scopes enable the visualisation of both angle and speed error. The main scope is plotting the joints position and speed in 𝑑𝑒𝑔 and 𝑑𝑒𝑔/𝑠, the desired torque and the perturbation one.
In principle, no change has to be done inside the block. All the configuration parameters and trajectories definition are inside the initialisation.m file. There is the possibility to load a trajectory as a reference (𝑐=0) or to simulate steps, ramps or sine waves (𝑐=1). The meaning of each parameter is explained in details in the file.
