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
% setenv('YARP_ROBOT_NAME', 'icubGazeboSim');


ROBOT_DOF = 19;
Kp        = 1/10*diag( ones(1,ROBOT_DOF)  );


robotName = 'icub';
localName = 'impedance';
Ts        = 0.01;
% Kp        = 0.01;
Kd        = 0.0001;