# iCub on a seesaw

This controller aims to balance the humanoid robot iCub while standing on a moving platform, i.e. the seesaw board.

# Structure of the repository:

- [simulink](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/seesaw/controllers/torqueBalancingOnSeesaw/simulink)
- [imuCalib](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/seesaw/controllers/torqueBalancingOnSeesaw/imuCalib)

# simulink

This folder contains the Simulink model and Matlab initialization files for balancing controller.

### how to run a simulation:

- install [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) repository and follow the [seesaw README](https://github.com/robotology-playground/icub-gazebo-wholebody/blob/master/worlds/icub_seesaw_world/README.md) instruction for opening the model in Gazebo.
- run `yarpserver` on a terminal.
- run Gazebo on another terminal by using the command you can find [here](https://github.com/robotology-playground/icub-gazebo-wholebody/blob/master/worlds/icub_seesaw_world/README.md).
- it is also necessary to run the estimator thread. In simulation, open a terminal and type `YARP_ROBOT_NAME=<name of your robot> yarprobotinterface --config launch-wholebodydynamics.xml`. Further information can be found [here](https://github.com/robotology/codyco-modules/blob/master/doc/force_control_on_icub.md#run-wholebodydynamics-on-an-external-pc).
- set your simulation preferences in the [initialization file](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/seesaw/controllers/torqueBalancingOnSeesaw/simulink/initTrqBalancingSeeSaw.m) (in particular, select the proper `YARP_ROBOT_NAME` according to the robot you are using. 
- open the [simulink model](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/seesaw/controllers/torqueBalancingOnSeesaw/simulink/controller.mdl) and let it run!

# imuCalib 

A simple Simulink model for verifying IMU measurements. Connect the FOG IMU to your PC by following the instructions 
in [kvh-yarp-devices](https://github.com/robotology-playground/kvh-yarp-devices) repositiory and then run the model.
