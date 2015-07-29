%% icubGazeboSim
ROBOT_DOF = 1;
robotName = 'icub';
localName = 'impedance';
Ts        = 0.01;

KpTorso   = 0.5*ones(1,3);
KpArms    = [0.2,0.2,0.2,0.05,0.1];
KpLegs    = [0.2,0.2,0.2,0.05,0.1,0.1];
Kp        = diag([1]);

if size(Kp,1) ~= ROBOT_DOF
    error('Dimension of Kp different from ROBOT_DOF')
end

Kd        = 2*sqrt(Kp)*0;

GRAV_COMP = 1;
MOVING    = 1;
AMPLS     = 10*ones(1,ROBOT_DOF);
FREQS     = 0.5*ones(1,ROBOT_DOF);

if MOVING == 1
    simulationTime = 5/FREQS(1);
else
    simulationTime = inf;
end