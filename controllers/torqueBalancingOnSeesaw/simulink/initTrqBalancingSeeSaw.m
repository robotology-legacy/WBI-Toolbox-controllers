clear all;
setenv('YARP_ROBOT_NAME','icubGazeboSim');

localName               = 'seesawBalancingController';
Ts = 0.01;
seesawKind              = 2;

seesaw_inertial         = '/seesaw/inertial';
CONFIG.CONTROLKIND      = 1;
CONFIG.USE_IMU4EST_BASE = 0;

CONFIG.USE_IMU4EST_BASE = false;
CONFIG.YAW_IMU_FILTER   = false;

CONFIG.TS               = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
addpath('../../utilityMatlabFunctions/')
CONFIG.ON_GAZEBO     = false;
WBT_modelName = 'matlabTorqueBalancing';

run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);
