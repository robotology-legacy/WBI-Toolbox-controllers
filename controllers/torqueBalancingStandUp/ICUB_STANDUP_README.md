## ICUB STANDUP DEMO: INSTRUCTIONS

### Dependencies

In order to run the simulation of iCub standing up from a chair, make sure you have installed on your pc the following dependencies:

 - [yarp](https://github.com/robotology/yarp)
 - [icub-main](https://github.com/robotology/icub-main)
 - [Gazebo simulator](http://gazebosim.org/)
 - [codyco-superbuild](https://github.com/robotology/codyco-superbuild) 

 - Both iCub and the chair models are stored into [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) repository.
 - [WBToolbox](https://github.com/robotology/WB-Toolbox) and [WBIToolbox-controllers](https://github.com/robotology-playground/WBI-Toolbox-controllers) are required for setting up the simulation.

 It is suggested to install the above mentioned repositories using codyco-superbuild (enable `CODYCO_USES_GAZEBO`, `CODYCO_USES_MATLAB`, `CODYCO_USES_WBI_TOOLBOX_CONTROLLERS` options).
 
### Installation

1) In your `.bashrc` file, add the following line to your `GAZEBO_RESOURCE_PATH`:
    
  ```
    source /usr/share/gazebo/setup.sh
    export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/PATH/WHERE/YOUR/WORLD/IS
   
  ```
 
  Instead of opening Gazebo as usual, on a terminal type:

   `gazebo -slibgazebo_yarp_clock.so nameOfYourWorld` where for iCub standup demo `nameOfYourWorld = icub_standup_world`.

   and it will load the correct model.

### Simulations

- To run iCub stand up demo, just use torqueBalancing controller as for the other simulations. (for further details see [README](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/master/controllers/torqueBalancing/README.md)). 
  Do not open the yarpmotorgui: iCub is already in its home position!

- In the [initialization file](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/master/controllers/torqueBalancingStandUp/initTorqueBalancing.m), there is now the new state-machine option "STANDUP" as long as the old ones "YOGA" and "COORDINATOR".

- All the demos "STANDUP", "YOGA" and "COORDINATOR" should be still available without conflicts (BUT with the right Gazebo environment for each demo). 
