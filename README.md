### Warning: this repository is not mantained
The Simulink controllers contained in the repo are not supposed to work. This repository is currently used as `legacy` of [whole-body-controllers](https://github.com/robotology/whole-body-controllers), see also https://github.com/robotology/whole-body-controllers/blob/master/README.md

# WBI-Toolbox-controllers
Controllers developed using both the WBI-Toolbox and the WB-Toolbox.
In the folder which correspond to each controller the user can find a detailed README. Here we present a short 
description of each balancing controller and the README links.

## Available models 

All the models implemented with the old WBI-Toolbox are in the [legacy](controllers/legacy) folder. The
new [WB-Toolbox](https://github.com/robotology/WB-Toolbox) models are:

- [boundedTorqueControl](#boundedtorquecontrol)
- [torqueBalancing](#torquebalancing)
- [legacy](#legacy)
- [torqueBalancingOnSeesaw](#torquebalancingonseesaw)
- [torqueBalancingStandUp](#torquebalancingstandup)
- [wholeBodyImpedanceControl](#wholebodyimpedancecontrol)

Check the documentation in [WB-Toolbox](https://github.com/robotology/WB-Toolbox) for installation of the Simulink toolbox.

### torqueBalancing
This simulink model controls the robot using a momentum-based balancing control.
The user can find the full description here: [torqueBalancing README](controllers/torqueBalancing/README.md)

### boundedTorqueControl

### legacy
A folder containing old Simulink controllers for balancing.

### torqueBalancingOnSeesaw
This is a peculiar version of momentum-based balancing controller, specifically developed for balancing the robot on a seesaw. The readme is at the following link: [torqueBalancingOnSeesaw README](controllers/torqueBalancingOnSeesaw/README.md).

### torqueBalancingStandUp
Another peculiar version of momentum-based balancing controller designed for standing up from a chair. See relative [torqueBalancingStandUp README](controllers/torqueBalancingStandUp/ICUB_STANDUP_README.md).

###  wholeBodyImpedanceControl
A simple impedance controller for the robot balancing. 

## Add a new model
To add a new model to the repo, please follow the [guidelines on model creation](doc/model_guidelines.md).
