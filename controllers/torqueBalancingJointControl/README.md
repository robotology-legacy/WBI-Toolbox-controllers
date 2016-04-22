##Module description

This module implements a torque control balancing strategy.
It computes directly the control torques which stabilize the joint space dynamics. The joint references are estimated using a simple inverse-kinematics 
solver, which is structured as a prioritized inverse kinematics integrator.

###Compatibility
This Simulink model is compatible with Matlab 2012b or any later version.

###Launch procedure
The procedure to run the torque balancing module is still quite elaborate.
Users willing to use the module should follow this list.

- Set the environmental variable YARP_ROBOT_NAME in the bashrc according to the robot one wants to 
   use (e.g. icubGazeboSim for simulations, or iCubGenova01, etc. for experiments).
- (Simulation only) Launch yarpserver.
- (Simulation only) Launch gazebo. If you want to use the synchronization between the controller and the simulator to avoid real-time factor 
   related problems, launch gazebo as follows: `gazebo -slibgazebo_yarp_clock.so` 
- Bring the robot in a suitable home position (e.g. `$ yarpmotorgui --from homePoseBalancingTwoFeet.ini` and then pressing the 'Home All' button)
-  Launch `wholeBodyDynamicsTree` as follows: `$ wholeBodyDynamicsTree --autoconnect --robot YARP_ROBOT_NAME_VALUE`
- Execute the [sensors calibration script](https://github.com/robotology/codyco-modules/blob/master/src/scripts/twoFeetStandingIdleAndCalib.sh): 
  `$ twoFeetStandingIdleAndCalib.sh`
- Open the simulink model `torqueBalancingJointControl.mdl`
- Run the module 

##Module details
###Configuration file
At start, the module calls the initialization file initTorqueBalancing.m. Once open, this file contains some configuration variables.
####General section
- `robotName : name of the robot to connect to, i.e. 'icubGazeboSim' for simulations, or 'icub' for experiments`
- `localName`: module name. Ports will be opened with this name. Default to `matlabTorqueBalancing`
- `simulationTime`: time of the simulation/experiment
- `USE_QP_SOLVER`: if 1, then a QP will be used to generate torques. If 0, classical pseudo-inverse are used
- `LEFT_RIGHT_FOOT_IN_CONTACT`: this two-element vector specifies which foot is in contact with the ground at the start fo the module. [1 1]: 
   both feet in contact, [1 0] left foot in contact; [0 1] right foot in contact;  
- `DEMO_MOVEMENTS`: if equal to 1 and the robot is standing on double support, then it will perform the classical left-and-right.  

####Gains
The gains for the robot specified by the variable YARP_ROBOT_NAME can be found in the folder `robots/YARP_ROBOT_NAME/gains.m`. This file contains
several gains, among which

- `maxTorque`: saturations to be applied to the output torques. It must be a positive value.
- `gain.ikin.kp`: proportional gains for inverse kin. 3 values
- `gain.ikin.kd`: derivative gains for inverse kin. 3 values
- `impTorso, impArms, impLeftLeg, impRightLeg`: gains for the impedance (low level) task. The number must match the number of torque controlled joints.

The latter five gains can be chosen differently for one foor, or two feet balancing.

Please consider that this controller has never been tested with the robot balancing on 1 foot.


