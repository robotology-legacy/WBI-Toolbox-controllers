clear;% all;
clc;

setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','icubGazeboSim');

CONFIG.SIMULATION_TIME      = inf;    % Simulation time in seconds

CONFIG.USE_QP_SOLVER       = 1;


CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

CONFIG.DEMO_MOVEMENTS      = false; % Either true or false 

CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                            % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                            % of the postural tasks are smoothed internally 
                            
CONFIG.USE_IMU4EST_BASE    = false;


CONFIG.YAW_IMU_FILTER      = false;

% PLEASE, use logical values (true or false) for the following variable
                            
SM.SM_TYPE                = 'WALKING'; % 'YOGA' or 'WALKING', or COORDINATOR
                           
CONFIG.Ts                 = 0.01; %  Controller period [s]


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
CONFIG.ON_GAZEBO     = false;
WBT_modelName = 'matlabTorqueBalancing';

run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('extra/')
[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);


robotSpecificReferences = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initRefGen.m');
run(robotSpecificReferences);

SM.SM.MASK.COORDINATOR = bin2dec('001');
SM.SM.MASK.YOGA = bin2dec('010');
SM.SM.MASK.WALKING = bin2dec('100');

SM.SM_TYPE_BIN = SM.SM.MASK.COORDINATOR;
robotSpecificFSM = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initStateMachine.m');
run(robotSpecificFSM);

if strcmpi(SM.SM_TYPE, 'COORDINATOR')
    SM.SM_TYPE_BIN = SM.SM.MASK.COORDINATOR;
elseif strcmpi(SM.SM_TYPE, 'YOGA')
    SM.SM_TYPE_BIN = SM.SM.MASK.YOGA;
elseif strcmpi(SM.SM_TYPE, 'WALKING')
    SM.SM_TYPE_BIN = SM.SM.MASK.WALKING;
    robotSpecificFSM = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initStateMachineWalking.m');
end
run(robotSpecificFSM);


% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulonk model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".
