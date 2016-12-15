## ICUB ON A CHAIR DEMO

### Instructions

1) In the `WBIToolbox-controllers` repository, checkout the branch `icub_chair`.

2) In your `.bashrc` file, add the following line:
    
  `alias gazebo_chair="cd ~/$CODYCO_ROOT/build/install/share/gazebo/worlds/icub_chair_world && gazebo -slibgazebo_yarp_clock.so`
 
   just to initialize the Gazebo world with iCub on a chair. 

3) Then, instead of opening Gazebo as usual (`gazebo -slibgazebo_yarp_clock.so icub_chair_world`) on a terminal just type:

   `gazebo_chair`

### To run the simulation, just run the torqueBalancing controller as usual. In the initialization file, there will be the new option 
    "CHAIR" as long as the old ones "YOGA" and "COORDINATOR".


### All the demos "CHAIR", "YOGA" and "COORDINATOR" should be still available without conflicts 
    (BUT with the right gazebo world for each demo). 
