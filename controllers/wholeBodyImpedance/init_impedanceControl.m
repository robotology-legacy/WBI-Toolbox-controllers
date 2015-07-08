%% icubGazeboSim
ROBOT_DOF = 13;
robotName = 'icub';
localName = 'impedance';
Ts        = 0.01;
simulationTime = inf;

KpTorso   = 0.5*ones(1,3);
KpArms    = [0.2,0.2,0.2,0.05,0.1];
KpLegs    = [0.2,0.2,0.2,0.05,0.1,0.1];
Kp        = diag([KpTorso,KpArms,KpArms]);

if size(Kp,1) ~= ROBOT_DOF
    error('Dimension of Kp different from ROBOT_DOF')
end

Kd        = 0.0001;

GRAV_COMP = 1;
MOVING    = 1;
AMPLS     = 10*ones(1,ROBOT_DOF);
FREQS     = 1/1*ones(1,ROBOT_DOF);