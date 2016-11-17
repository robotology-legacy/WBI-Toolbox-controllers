## ICUB ON A CHAIR DEMO

### Instructions

1) in `icub-gazebo` repository, checkout the branch `icub_chair`

2) in `codyco-superbuild` repository, checkout the branch `demo_codyco` in both 
   `yarpWholeBodyInterface` and `WBIToolboxControllers` (and build the new repositories)

3) in the `.bashrc` file, add the following line:
    
  `alias gazebo_chair="cd ~/path/to/icub-gazebo/icub_chair_world && gazebo -slibgazebo_yarp_clock.so icub_chair_world"`
 
   just to initialize the Gazebo world with iCub on a chair. 

4) then, instead of opening Gazebo as usual (`gazebo -slibgazebo_yarp_clock.so icub_chair_world`) on a terminal just type:

   `gazebo_chair`


### To run the simulation, just run the torqueBalancing controller as usual. In the initialization file, there will be the new option 
    "CHAIR" as long as the old ones "YOGA" and "COORDINATOR".


### All the demos "CHAIR", "YOGA" and "COORDINATOR" should be still available without conflicts 
    (BUT with the right gazebo world for each demo). 
