clear all;

robotName = 'icub';            
localName = 'balancing';

simulationTime    = inf;       % Simulation time in seconds

number_of_feet_on_ground = 1;  % Either 1 or 2

DEMO_LEFT_AND_RIGHT      = 0;  % Either 0 or 1 
noOscillationTime        = 0; % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                               % that the robot waits before starting the left-and-righ

DEMO_MOVING_LEG_AND_ARMS = 0; 

 

% Controller period
Ts                = 0.01; % [s]

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

