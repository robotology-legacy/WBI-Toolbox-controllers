clear;
setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubDarmstadt01');

localName               = 'seesawBalancingController';
Ts = 0.01;
seesawKind              = 2;

seesaw_inertial         = '/seesaw/inertial';
CONFIG.CONTROLKIND      = 1;

CONFIG.USE_ROBOT_IMU4SEESAW = false;

CONFIG.YAW_IMU_FILTER   = true;

CONFIG.PITCH_IMU_FILTER = false;

CONFIG.USE_QP_SOLVER    = true;
CONFIG.SCOPES           = true;

CONFIG.TS               = 0.01;

CONFIG.CONSIDERSEESAWDYN  = 1; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
WBT_modelName = 'matlabTorqueBalancingSeesaw';

addpath('../../utilityMatlabFunctions/')
addpath('./src')

CONFIG.ON_GAZEBO     = false;

PORTS.IMU       = '/icub/inertial';


run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);
