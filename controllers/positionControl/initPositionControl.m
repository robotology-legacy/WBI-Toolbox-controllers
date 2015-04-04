clear all;

robotName = 'icubGazeboSim';            
localName = 'positionControl';

simulationTime    = 5;       % Simulation time in seconds


% Controller period
Ts                = 0.01; % [s]

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

