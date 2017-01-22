clear;

% setenv('YARP_ROBOT_NAME','icubGazeboSim');
setenv('YARP_ROBOT_NAME','iCubGenova02');

localName                  = 'seesawBalancingController';
Ts                         = 0.01;

seesaw_inertial            = '/seesaw/inertial';
CONFIG.CONTROLKIND         = 4;

CONFIG.USE_QP_SOLVER       = true;
CONFIG.SCOPES              = true;
CONFIG.CORRECT_NECK_IMU    = true;
CONFIG.TS                  = 0.01;

CONFIG.CONSIDERSEESAWDYN   = 1; 

% Used for the new integral of angular momentum estimation
LEFT_RIGHT_FOOT_IN_CONTACT = [1;1];
CONFIG.USE_IMU4EST_BASE    = true;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
WBT_modelName        = 'matlabTorqueBalancingSeesaw';

addpath('../../utilityMatlabFunctions/')
addpath('./src')

CONFIG.ON_GAZEBO     = false;

PORTS.IMU            = '/icub/inertial';
PORTS.NECK           = '/icub/head/state:o';

seesawKind           = 2;

run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/initGains.m')); 
[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);

CONFIG.USE_ROBOT_IMU4SEESAW = true;
CONFIG.YAW_IMU_FILTER       = true;
CONFIG.PITCH_IMU_FILTER     = true;
