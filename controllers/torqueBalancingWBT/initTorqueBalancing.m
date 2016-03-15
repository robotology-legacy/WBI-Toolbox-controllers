clear;% all;
clc;

% setenv('YARP_ROBOT_NAME','iCubGenova02');
setenv('YARP_ROBOT_NAME','icubGazeboSim');


CONFIG.SIMULATION_TIME     = inf;    % Simulation time in seconds

SM.SM_TYPE                 = 'WALKING';   % 'YOGA' or 'WALKING', or COORDINATOR

CONFIG.USE_QP_SOLVER       = 1;
                            
CONFIG.USE_IMU4EST_BASE    = false;
CONFIG.YAW_IMU_FILTER      = false;
                            
                           
CONFIG.Ts                  = 0.01; %  Controller period [s]


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
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

%% Loading walking trajectories
WALKING_TRAJ_SRC_DIR = fullfile([getenv('CODYCO_SUPERBUILD_DIR') '/install/share/codyco/contexts/walkPlayer']);
COM_DES_FILE         = [WALKING_TRAJ_SRC_DIR '/torqueBalancing_comTraj.txt'];
QJ_DES_FILE          = [WALKING_TRAJ_SRC_DIR '/torqueBalancing_posturalTraj.txt'];
CONSTRAINTS_DES_FILE = [WALKING_TRAJ_SRC_DIR '/torqueBalancing_constraints.txt'];

COM_DES = importdata(COM_DES_FILE, '\t');
COMDES = str2num(cell2mat(COM_DES(1:end-1, 1)));
COM_DES = timeseries(COMDES, [0:0.010:0.010*(length(COMDES)-1)]); 
QJ_DES = importdata(QJ_DES_FILE, '\t');
QJDES = str2num(cell2mat(QJ_DES(1:end-1, 1)));
QJ_DES = timeseries(QJDES, [0:0.010:0.010*(length(QJDES)-1)]); 
CONSTRAINTS_DES = importdata(CONSTRAINTS_DES_FILE,'\t');
CONSTRAINTSDES = str2num(cell2mat(CONSTRAINTS_DES(1:end-1, 1)));
CONSTRAINTS_DES = timeseries(CONSTRAINTSDES, [0:0.010:0.010*(length(CONSTRAINTSDES)-1)]);

% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulink model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".
