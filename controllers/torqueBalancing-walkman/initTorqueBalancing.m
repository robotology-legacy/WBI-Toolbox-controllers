%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
%% GENERAL SIMULATION INFO
% If you are simulating the robot with Gazebo, 
% remember that you have to launch Gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
% 
% and set the environmental variable YARP_ROBOT_NAME = icubGazeboSim.
% To do this, you can uncomment the 

% setenv('YARP_ROBOT_NAME','iCubGenova01');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','iCubDarmstadt01');
% setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova05');
setenv('YARP_ROBOT_NAME','bigman');
% setenv('YARP_ROBOT_NAME','bigman_only_legs');

CONFIG.ON_WALKMAN_PILOT_PC = false;
CONFIG.ON_REAL_WALKMAN     = false;

% dynamic calibration parameters
USE_h_ONLY         = true;
USE_JOINT_VELOCITY = 1;
tSwitch            = [10 20 30 40 50 60 70 80 90 100];
tEnd               = tSwitch(end) + 10;

% Simulation time in seconds
CONFIG.SIMULATION_TIME = inf;

% Available movesets: 1 = air_1; 2 = air_2, 3 = yoga
CONFIG.moveset = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%% SET ENVIRONEMENTAL VARIABLES (only for walkman-pilot-pc) %%%%%% %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if CONFIG.ON_WALKMAN_PILOT_PC

    setenv('CODYCO_SUPERBUILD_ROOT','/home/lucamuratore/src/codyco-superbuild');
    current_path = getenv('PATH');
    setenv('PATH',fullfile(current_path, ':/home/lucamuratore/src/codyco-superbuild/build/install/bin'));
    current_ld_library_path = getenv('LD_LIBRARY_PATH');
    setenv('LD_LIBRARY_PATH',fullfile(current_ld_library_path, ':home/lucamuratore/src/codyco-superbuild/build/install/lib'));
    setenv('YARP_DATA_DIRS','/home/lucamuratore/src/codyco-superbuild/build/install/share/yarp:/home/lucamuratore/src/codyco-superbuild/build/install/share/iCub:/home/lucamuratore/src/codyco-superbuild/build/install/share/codyco');
end

% calibration delta for legs joints
newOffsets = [0.12753
              2.554397
              2.024109
             -2.243889
              1.644721
             -0.39407
             -0.642021
             -0.183969
              0.233145
             -1.176114
              0.866804
              0.247708]; % [deg]

% calibration delta (real - desired) 
calibDelta = newOffsets*pi/180;
PORTS.IMU_CALIB = '/bigman/imu_link/inertial';

if CONFIG.ON_REAL_WALKMAN == 0

    % ONLY FOR SIMULATION
    calibDelta = 0.*calibDelta;
    PORTS.IMU_CALIB = '/bigman/inertial';
end

if CONFIG.moveset == 3
   
    tSwitch  = [10,20,30,40];
    tEnd     = tSwitch(end) + 10;    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% PRELIMINARY CONFIGURATIONS 
% SM.SM_TYPE: defines the kind of state machines that can be chosen.
%
% 'YOGA': the robot will perform the YOGA++ demo. The associated
%         configuration parameters can be found under the folder
%
%         robots/YARP_ROBOT_NAME/initStateMachine.m
%   
% 'COORDINATOR': the robot can either stay still, or follow a
%                center-of-mass trajectory, or follow references for the
%                joints. The associated configuration parameters can be 
%                found under the folder
%
%               robots/YARP_ROBOT_NAME/initRegGen.m
% 
% 'WALKING': under development.
SM.SM_TYPE                   = 'COORDINATOR';

% CONFIG.SCOPES: if set to true, all visualizers for debugging are active
CONFIG.SCOPES.ALL            = true;

% You can also activate only some specific debugging scopes
CONFIG.SCOPES.BASE_EST_IMU   = false;
CONFIG.SCOPES.EXTWRENCHES    = false;
CONFIG.SCOPES.GAIN_SCHE_INFO = false;
CONFIG.SCOPES.MAIN           = false;
CONFIG.SCOPES.QP             = false;

% CONFIG.CHECK_LIMITS: if set to true, the controller will stop as soon as 
% any of the joint limit is touched. 
CONFIG.CHECK_LIMITS        = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, 
WBT_modelName            = 'matlabTorqueBalancing';
FRAMES.BASE              = 'root_link'; 
FRAMES.IMU               = 'imu_frame';

% CONFIG.USE_IMU4EST_BASE: if set to false, the base frame is estimated by 
% assuming that either the left or the right foot stay stuck on the ground. 
% Which foot the  controller uses depends on the contact forces acting on it. 
% If set to true, the base orientation is estimated by using the IMU, while
% the base position by assuming that the origin of either the right or the
% left foot do not move. 
CONFIG.USE_IMU4EST_BASE    = true;

% CONFIG.YAW_IMU_FILTER and CONFIG.PITCH_IMU_FILTER: when the flag
% CONFIG.USE_IMU4EST_BASE = true, then the orientation of the floating base is
% estimated as explained above. However, the foot is usually perpendicular
% to gravity when the robot stands on flat surfaces, and rotation about the
% gravity axis may be de to the IMU drift in estimating this angle. Hence,
% when either of the flags CONFIG.YAW_IMU_FILTER or CONFIG.PITCH_IMU_FILTER
% is set to true, then the yaw and/or pitch angles of the contact foot are
% ignored and kept equal to the initial values.
CONFIG.YAW_IMU_FILTER      = false;
CONFIG.PITCH_IMU_FILTER    = false;

% CONFIG.CORRECT_NECK_IMU: when set equal to true, the kineamtics from the
% IMU and the contact foot is corrected by using the neck angles. If it set
% equal to false, recall that the neck is assumed to be in (0,0,0)
CONFIG.CORRECT_NECK_IMU    = true;

% CONFIG.ONSOFTCARPET: the third year CoDyCo review meeting consisted also
% of a validation scenarion in which the robot had to balance on a soft
% carpet. Hence, when CONFIG.ONSOFTCARPET = true, other sets of gains are
% loaded for the postural and CoM.
CONFIG.ONSOFTCARPET        = false;

% CONFIG.USE_QP_SOLVER: if set to true, a QP solver is used to account for 
% inequality constraints of contact wrenches
CONFIG.USE_QP_SOLVER       = true; 

PORTS.IMU       = '/icub/inertial';

PORTS.COM_DES   = ['/' WBT_modelName '/comDes:i'];

PORTS.Q_DES     = ['/' WBT_modelName '/qDes:i'];

PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamicsTree/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamicsTree/right_leg/cartesianEndEffectorWrench:o';

CONFIG.Ts                = 0.01; %  Controller period [s]

CONFIG.ON_GAZEBO         = false;
baseToWorldRotationPort  = ['/' WBT_modelName '/floatingBaseRotationMatrix:i'];

run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('./src/')
addpath('../utilityMatlabFunctions/')

robotSpecificReferences  = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initRefGen.m');
run(robotSpecificReferences);

SM.SM.MASK.COORDINATOR   = bin2dec('001');
SM.SM.MASK.YOGA          = bin2dec('010');
SM.SM.MASK.WALKING       = bin2dec('100');

SM.SM_TYPE_BIN = SM.SM.MASK.COORDINATOR;
robotSpecificFSM = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachine.m');
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

[ConstraintsMatrix,bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);


