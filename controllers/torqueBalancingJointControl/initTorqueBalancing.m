clear;
clc;

% setenv('YARP_ROBOT_NAME','iCubGenova02');
  setenv('YARP_ROBOT_NAME','icubGazeboSim');

CONFIG.SIMULATION_TIME             = inf;      % Simulation time in seconds
CONFIG.USE_QP_SOLVER               = 0;      
CONFIG.Ts                          = 0.01;    % Controller period [s]                     
CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];
CONFIG.DEMO_MOVEMENTS              = 1;       % Either 0 or 1 

%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO
WBT_modelName            = 'matlabTorqueBalancing';

run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

addpath('./matlab-src/')

[ConstraintsMatrix,bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);

robotSpecificReferences = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initRefGen.m');

run(robotSpecificReferences);


% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulink model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".
