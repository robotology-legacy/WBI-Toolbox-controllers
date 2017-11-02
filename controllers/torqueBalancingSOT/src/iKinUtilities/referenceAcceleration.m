function dnuDes   = referenceAcceleration(Jtask, dJnuDes, taskAccelerations, jointDynamics, ... 
                                          ROBOT_DOF, reg)
                                          
% Generate the reference accelerations for both the floating base and the joints through a stack of task inverse kinematics structure

% setup parameters
PINV_TOL          = reg.pinvDampVb;     
    
%% Stack-of-tasks inverse kinematics
    
%Primary task: respect task constraints
CoMrootAcc = taskAccelerations(1:6);
JCoMroot   = Jtask(1:6, :);
dnuCoMroot = pinv(JCoMroot, PINV_TOL) * (CoMrootAcc - dJnuDes(1:6));
nullCoMroot= eye(6 + ROBOT_DOF) - pinv(JCoMroot, PINV_TOL) * JCoMroot;

%Secondary task: Feet contact
feetAcc     = taskAccelerations(7:end);
Jfeet       = Jtask(7:end, :);
dJnu_feet   = dJnuDes(7:end);
dnu_feet    = pinv(Jfeet, PINV_TOL) * (feetAcc - dJnu_feet);
null_feet   = eye(6 + ROBOT_DOF) - pinv(Jfeet, PINV_TOL) * Jfeet;

%Tertiary task: joint posture
S           = [zeros(ROBOT_DOF, 6) eye(ROBOT_DOF)];
dnuPosture  = pinv(null_feet, PINV_TOL) * (pinv(S * nullCoMroot, PINV_TOL) * (jointDynamics - S * dnuCoMroot) - dnu_feet);

dnuDes = dnuCoMroot + nullCoMroot * (dnu_feet + null_feet * dnuPosture);



end