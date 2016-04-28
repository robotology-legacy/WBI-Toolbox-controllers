# WBI-Toolbox-controllers
Controllers developed using both the WBI-Toolbox and the WB-Toolbox.
In the folder which correspond to each controller the user can find a detailed README. Here we present a short 
description of each balancing controller and the README links.

All the models implemented with the old WBI-Toolbox are in the [legacy](controllers/legacy) folder. The
new WB-Toolbox models are:

- [torqueBalancing](#torquebalancing)
- [torqueBalancing_experiments](#torquebalancing_experiments)
- [torqueBalancingOnSeesaw](#torquebalancingonseesaw)
- [wholeBodyImpedanceControl](#wholebodyimpedancecontrol)

## torqueBalancing
This simulink model controls the robot using a momentum-based balancing control.
The user can find the full description here: [torqueBalancing README](controllers/torqueBalancing/README.md)

## torqueBalancing_experiments
This folder contains two modifications of the original torqueBalancing code. These versions are deprecated and will be removed soon.

## torqueBalancingOnSeesaw
This is a peculiar version of momentum-based balancing controller, specifically developed for balancing the robot on a seesaw. The readme is at the following link: [torqueBalancingOnSeesaw README](controllers/torqueBalancingOnSeesaw/README.md).

## wholeBodyImpedanceControl
A simple impedance controller for the robot balancing. 
