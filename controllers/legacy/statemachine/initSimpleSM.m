clear;% all;
clc;

% setenv('YARP_ROBOT_NAME','iCubGenova02');
setenv('YARP_ROBOT_NAME','icubGazeboSim');
setenv('YARP_DATA_DIRS','/Users/makaveli/Projects/src/local/share/yarp:/Users/makaveli/Projects/src/local/share/iCub:/Users/makaveli/Projects/src/local/../codyco-superbuild/build/install/share/codyco');
setenv('DYLD_LIBRARY_PATH','/Users/makaveli/Projects/src/codyco-superbuild/build/install/lib');

CONFIG.SIMULATION_TIME      = inf;    % Simulation time in seconds

CONFIG.USE_IMU4EST_BASE    = false;

CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

CONFIG.DEMO_MOVEMENTS      = false; % Either true or false 

CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                            % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                            % of the postural tasks are smoothed internally 


% PLEASE, use logical values (true or false) for the following variable

CONFIG.USE_SM             = true;  % If equal to true, the (internal) state machine 
                            % will be used. The robot will switch from 2 feet 
                            % to 1 (left) foot
                           
CONFIG.Ts                 = 0.01; %  Controller period [s]


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
CONFIG.ON_GAZEBO     = false;
WBT_modelName = 'matlabTBStateMachine';

run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('extra/')

robotSpecificFSM = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initStateMachine.m');
run(robotSpecificFSM);



% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulonk model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".
