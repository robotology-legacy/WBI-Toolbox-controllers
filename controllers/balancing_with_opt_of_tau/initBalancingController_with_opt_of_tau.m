clear all

robotName = 'icubGazeboSim';
localName = 'balancing';

number_of_feet_on_ground = 2;
DEMO_LEFT_AND_RIGHT      = 0;
noOscillationTime        = 0;


DEMO_MOVING_LEG_AND_ARMS = 0;  

% Controller period
Ts                = 0.01;
simulationTime    = inf;

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

% run(strcat('robots/','iCubHeidelberg01','/gains.m')); 
