clear;
clc;

% setenv('YARP_ROBOT_NAME','iCubGenova02');
setenv('YARP_ROBOT_NAME','icubGazeboSim');


CONFIG.SIMULATION_TIME     = inf;    % Simulation time in seconds

SM.SM_TYPE                 = 'COORDINATOR';   % 'YOGA' or 'WALKING', or COORDINATOR

CONFIG.USE_CENTR_ORIENT    = 1;
CONFIG.USE_QP_SOLVER       = 0;
                            
CONFIG.USE_IMU4EST_BASE    = false;
CONFIG.YAW_IMU_FILTER      = false;
                            
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, 
%  i.e. YARP_ROBOT_NAME=icubGazeboSim

CONFIG.Ts                  = 0.01; %  Controller period [s]

CONFIG.ON_GAZEBO     = false;
WBT_modelName = 'matlabTorqueBalancing';
baseToWorldRotationPort  = ['/' WBT_modelName '/floatingBaseRotationMatrix:i'];

dump.left_wrench_port = '/wholeBodyDynamicsTree/left_foot/cartesianEndEffectorWrench:o';
dump.right_wrench_port = '/wholeBodyDynamicsTree/right_foot/cartesianEndEffectorWrench:o';

run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('../utilityMatlabFunctions/')
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
    run(robotSpecificFSM);
end


% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulink model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".
