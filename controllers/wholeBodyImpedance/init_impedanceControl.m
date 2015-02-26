%% icubGazeboSim
% setenv('YARP_ROBOT_NAME', 'icubGazeboSim');

ROBOT_DOF = 1;
Kp        = 1/10*diag( ones(1,ROBOT_DOF)  );


robotName = 'icub';
localName = 'impedance';
Ts        = 0.01;
Kd        = 0.0001;