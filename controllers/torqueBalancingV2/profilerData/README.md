Last updated on: Wednesday July 23, 2014
By: Jorhabib Eljaik

In this directory you will find the tests done for balancingController with the Simulink Profiler active during the iCub Summer School 2014.
In order to see the results of the profiler load the desired .mat file in the workspace and launch:

slprofreport(name_of_the_stored_profiler_data);

The files you will find here are profiler results when launching balancingController with the following setup:
a. wholeBodyDynamicsTree on /icub15 with the following parameters launched as:
    wholeBodyDynamicsTree --robot icub --headV2 --legsV2 --feetv2 --autoconnect --name wholeBodyDynamicsTree
b. jointTorqueControl on Daniele's computer launched as:
    jointTorqueControl --from iCubGenova01_Conf_file_for_JTC_whole_body_balancing.ini
c. Matlab launched on Daniele's computer as:
    matlab --nodesktop
d. balancingController.slx from /WBIToolbox/controllers/torqueBalancingV2/


The stored data is described next. In all the simulations the Stop Simulation Time was 60 seconds with a Fixed-step discrete solver and Ts = 0.010.

1. balancingControllerProfileDataNoSendTorques.mat   : controller without the sendTorques block.
2. balancingControllerProfileDataWithSendTorques.mat : controller with the sendTorques block.

Without the profiler it takes roughly 77 seconds to complete 60 simulation seconds which corresponds to a period of 17.8 ms
