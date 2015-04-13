clear all

robotName = 'icubGazeboSim';
localName = 'matlabTrqBalancingExtFsm';


% Controller period
Ts                = 0.01;
simulationTime    = inf;

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

