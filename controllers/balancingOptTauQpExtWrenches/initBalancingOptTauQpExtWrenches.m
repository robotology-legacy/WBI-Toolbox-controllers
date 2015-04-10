%WBC ASW
%  clear all
 
robotName = 'icubGazeboSim';
localName = 'balancing';


simulationTime      = inf;
noOscillationTime   = 0;
DEMO_LEFT_AND_RIGHT = 1;
 

% Controller period
Ts                = 0.01; % [s]

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
