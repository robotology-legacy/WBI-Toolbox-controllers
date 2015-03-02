clear all
 
YARP_ROBOT_NAME = getenv('YARP_ROBOT_NAME');

robotName = 'icub';
localName = 'balancing';
ROBOT_DOF = 23;

number_of_feet_on_ground          = 2;
DEMO_LEFT_AND_RIGHT               = 1; 

DEMO_MOVING_LEG_AND_ARMS = 0;  % It is taken into account only if number_of_feet_on_ground = 1


% Controller period
Ts                = 0.01;
simulationTime    = inf;

run(strcat('robots/',YARP_ROBOT_NAME,'/gains.m')); %Load gains for the specific robot

if (size(impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file initBalancingController_with_opt_of_tau.m');
end