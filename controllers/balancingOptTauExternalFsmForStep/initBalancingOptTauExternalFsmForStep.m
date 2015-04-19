clear all

robotName = 'icubGazeboSim';
localName = 'matlabTorqueBalancing';


% Controller period
Ts                = 0.01;
simulationTime    = inf;

STEP              = 0;

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

