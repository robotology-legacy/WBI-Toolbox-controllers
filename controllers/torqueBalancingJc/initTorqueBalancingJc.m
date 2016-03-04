clear;% all;
clc;
 
%  setenv('YARP_ROBOT_NAME','icubGazeboSim');
 setenv('YARP_ROBOT_NAME','iCubGenova02');

 
 WBT_modelName = 'matlabTorqueBalancing';
 
 WBT_wbiList   = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';

 simulationTime    = inf;       % Simulation time in seconds

 USE_QP_SOLVER     = 0;

 LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

 DEMO_MOVEMENTS    = 1;  % Either 0 or 1 

 % Controller period
 Ts                = 0.01; % [s]

 % Load gains and parameters for the specific robot
 run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
 addpath('extra/')
 [ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);

% Uncomment the following line if you want to sync Gazebo and simulink.
%      setenv('YARP_CLOCK','/clock');      
% If you want to sync Gazebo and simulink,
% you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulink model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".
