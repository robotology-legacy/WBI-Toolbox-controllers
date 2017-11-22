%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci & Marie Charbonneau
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
%clearing functions with persistent variables
clear stateMachineWalking QpBalancingSOT
%% GENERAL SIMULATION INFO
% If you are simulating the robot with Gazebo, 
% remember that you have to launch Gazebo as follows:
% 
% gazebo -slibgazebo_yarp_clock.so
% 
% and set the environmental variable YARP_ROBOT_NAME = icubGazeboSim.
% To do this, you can uncomment the 

%setenv('YARP_ROBOT_NAME','iCubGenova02');
% % setenv('YARP_ROBOT_NAME','iCubGenova04');
setenv('YARP_ROBOT_NAME','icubGazeboSim');

% Simulation time in seconds
CONFIG.SIMULATION_TIME     = inf;
% Simulation time step in seconds
CONFIG.Ts                  = 0.01; %  Controller period [s]

%% PRELIMINARY CONFIGURATIONS 
% SM.SM_TYPE: defines the kind of state machines that can be chosen. 
% 'WALKING': under development.
sm.SM_TYPE                 = 'WALKING';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, 
WBT_modelName            = 'matlabTorqueBalancing';

FRAMES.BASE              = 'root_link';

%Define which frame is to be used for the position task of the
%controller. Sensible choices include 'com'
FRAMES.POSITION_TASK  = 'com';

%Define which frame is to be used for the orientation task of the
%controller. Sensible choices include 'root_link', 'torso', 'head'
FRAMES.ORIENTATION_TASK  = 'head';

% CONFIG.USE_IMU4EST_BASE: if set to false, the base frame is estimated by 
% assuming that either the left or the right foot stay stuck on the ground. 
% Which foot the controller uses depends on the contact forces acting on it. 
% If set to true, the base orientation is estimated by using the IMU, while
% the base position by assuming that the origin of either the right or the
% left foot do not move. 
CONFIG.USE_IMU4EST_BASE    = false;

% CONFIG.YAW_IMU_FILTER and CONFIG.PITCH_IMU_FILTER: when the flag
% CONFIG.USE_IMU4EST_BASE = true, then the orientation of the floating base is
% estimated as explained above. However, the foot is usually perpendicular
% to gravity when the robot stands on flat surfaces, and rotation about the
% gravity axis may be de to the IMU drift in estimating this angle. Hence,
% when either of the flags CONFIG.YAW_IMU_FILTER or CONFIG.PITCH_IMU_FILTER
% is set to true, then the yaw and/or pitch angles of the contact foot are
% ignored and kept equal to the initial values.
CONFIG.YAW_IMU_FILTER      = true;
CONFIG.PITCH_IMU_FILTER    = true;

% CONFIG.CORRECT_NECK_IMU: when set equal to true, the kinematics from the
% IMU and the contact foot is corrected by using the neck angles. If it set
% equal to false, recall that the neck is assumed to be in (0,0,0)
CONFIG.CORRECT_NECK_IMU    = true;

%The balancing controller solves an optimization problem for desired tasks and posture
%There are at the moment 3 ways to define the balancing controller:
%1. Weighted/soft task priorities
%       minimizes weighted error on desired tasks and posture
%       use it by setting CONFIG.QP.USE_STRICT_TASK_PRIORITIES = false
%2. Strict task priorities
%       tasks defined as equality constraints on CoM position and root orientation
%       use it by setting CONFIG.QP.USE_STRICT_TASK_PRIORITIES = true
%3. Strict task priorities
%       tasks defined as equality constraints on CoM position, root orientation 
%       and feet pose (position and orientation)
%       use it by setting CONFIG.QP.USE_STRICT_TASK_PRIORITIES_WITH_FOOT_ACCELERATION = true
controllerType = 2;
switch controllerType
    case 1
        CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION   = false;
        CONFIG.QP.USE_STRICT_TASK_PRIORITIES_WITH_FOOT_ACCELERATION = false;
    case 2
        CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION   = true;
        CONFIG.QP.USE_STRICT_TASK_PRIORITIES_WITH_FOOT_ACCELERATION = false;
    case 3
        CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION   = false;
        CONFIG.QP.USE_STRICT_TASK_PRIORITIES_WITH_FOOT_ACCELERATION = true;
end

% CONFIG.QP.USE_CONTINUITY_CONSTRAINTS: when set to true, control torques
% obtained from QP optimization are constrained to continuous values.
CONFIG.QP.USE_CONTINUITY_CONSTRAINTS = false;

%CONFIG.USE_INVERSE_KINEMATICS: when set to true, use reference joint
%positions computed from integration-based inverse kinematics; when set to
%false, use desired joint positions from state machine.
CONFIG.USE_INVERSE_KINEMATICS = false;

% CONFIG.QP.USE_CENTROIDAL_TRANSFORMATION: when set to true, centroidal
% transformation is performed, in order to use the center of mass of the
% robot as base. This affects the computations performed, but shoud have 
% no incidence on the behavior of the robot.
CONFIG.QP.USE_CENTROIDAL_TRANSFORMATION = false;

%Impedance/damping gains can be set manually, or they can be tuned automatically
%using a perturbation-base extremum seeking (PES) algorithm.
CONFIG.USE_PES_gain_tuning = false;

% CONFIG.SCOPES.ALL: when set to false, all visualizations are disabled
CONFIG.SCOPES.ALL         = true;
% CONFIG.SCOPES.VALUE: when set to true, visualization of the element 
% in question is enabled
CONFIG.SCOPES.QP          = true;
CONFIG.SCOPES.TORQUES     = false;
CONFIG.SCOPES.JOINTS      = true;
CONFIG.SCOPES.FEET        = true;
CONFIG.SCOPES.TASKS       = false;
CONFIG.SCOPES.GAINS       = true;
%% 

addpath('./src/')
addpath('./src/iKinUtilities')
addpath('../utilityMatlabFunctions/')
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

robotSpecificFSM = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachineWalking.m');
run(robotSpecificFSM);

clear robotSpecificFSM