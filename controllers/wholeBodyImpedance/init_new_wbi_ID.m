%% DoublePendulum
% setenv('YARP_ROBOT_NAME', 'double_pendulum');
% ROBOT_DOF = 2;
% robotName = 'doublePendulumGazebo';
% localName = 'impedance';
% Ts        = 0.01;
% Kp        = 1.0;
% Kd        = 1.0;
% ROBOT_DOF = 2;
% robotName = 'doublePendulumGazebo';
% localName = 'impedance';
% Ts        = 0.01;
% Kp        = 1.0;
% Kd        = 1.0;

%% icubGazeboSim
setenv('YARP_ROBOT_NAME', 'icubGazeboSim');
ROBOT_DOF = 25;
robotName = 'icubGazeboSim';
localName = 'impedance';
Ts        = 0.01;
Kp        = 1/1000*diag( [ [10 6 6] 2*ones(1,10) 0.5*ones(1,12) ] );
% Kp        = 0.01;
Kd        = 0.0001;