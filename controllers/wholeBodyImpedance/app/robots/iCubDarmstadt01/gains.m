ROBOT_DOF   = 23;
% WBT_wbiList = 'ROBOT_UPPER_BODY';
WBT_wbiList = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';

robotName   = 'icub';
Ts          = 0.01;

KpTorso   = 0.0*ones(1,3);
KpArms    = [0.1,0.1,0.1,0.1];
KpLegs    = [0.1,0.1,0.1,0.1,0.1,0.1];
Kp        = diag([KpTorso,KpArms,KpArms])*0;
Kp        = zeros(ROBOT_DOF);

if size(Kp,1) ~= ROBOT_DOF
    error('Dimension of Kp different from ROBOT_DOF')
end

Kd        = 2*sqrt(Kp)*0;

AMPLS     = 15*ones(1,ROBOT_DOF);
FREQS     = 0.25*ones(1,ROBOT_DOF);