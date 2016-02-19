# WBI-Toolbox-controllers
Controllers developed using both the WBI-Toolbox and the WB-Toolbox.
In the folder which correspond to each controller the user can find a detailed README. Here we present a short 
description of each balancing controller and the README links.

Currently, the availabe balancing controllers are:
- [torqueBalancing and torqueBalancingWBT](#torquebalancing-and-torquebalancingwbt)
- [torqueBalancingJc and torqueBalancingJc_WBT](#torquebalancingjc-and-torquebalancingjc_wbt)
- [torqueBalancingOnSeesaw](#torquebalancingonseesaw)

## torqueBalancing and torqueBalancingWBT
These simulink models implement the momentum-based balancing control using the WBI-Toolbox and the WB-Toolbox,
respectively. The user can find their full description here: [torqueBalancing README](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/torqueBalancingJointControl/controllers/torqueBalancing/README.md), [torqueBalancingWBT README](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/torqueBalancingJointControl/controllers/torqueBalancingWBT/README.md).

## torqueBalancingJc and torqueBalancingJc_WBT
These controllers generate the control torques which stabilize directly the robot's joint space, without considering the centroidal dynamics. 
The joints references are obtained using an inverse kinematics algorithm. Their descriptions are here: [torqueBalancingJc README](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/torqueBalancingJointControl/controllers/torqueBalancingJc/README.md), [torqueBalancingJc_WBT README](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/torqueBalancingJointControl/controllers/torqueBalancingJc_WBT).

## torqueBalancingOnSeesaw
This is a peculiar version of momentum-based balancing controller, specifically developed for balancing the robot on a seesaw. The readme is at the following link: [torqueBalancingOnSeesaw README](https://github.com/robotology-playground/WBI-Toolbox-controllers/tree/torqueBalancingJointControl/controllers/torqueBalancingOnSeesaw/README.md).

