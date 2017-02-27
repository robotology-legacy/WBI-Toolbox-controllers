## ICUB ON A CHAIR DEMO

### Instructions

1) In the `WBIToolbox-controllers` repository, checkout the branch `icub_chair`.

2) In your `.bashrc` file, add the following line:
    
  `alias gazebo_chair="cd ~/YOUR/PATH/TO/icub_chair_world && gazebo -slibgazebo_yarp_clock.so`
 
   just to initialize the Gazebo world with iCub on a chair. 

3) Then, instead of opening Gazebo as usual (`gazebo -slibgazebo_yarp_clock.so icub_chair_world`) on a terminal just type:

   `gazebo_chair`

### Simulations
- To run the simulation, just run the torqueBalancing controller as usual, following the instructions in the [README](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/master/controllers/torqueBalancing/README.md). Do not open the yarpmotorgui: iCub is already in its home position!

- In the [initialization file](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/icub_chair/controllers/torqueBalancing/initTorqueBalancing.m), there is now the new simulation option "CHAIR" as long as the old ones "YOGA" and "COORDINATOR".

- All the demos "CHAIR", "YOGA" and "COORDINATOR" should be still available without conflicts (BUT with the right gazebo world for each demo). 
