function dnuDes   = referenceAcceleration(Jtask, dJnuDes, taskAccelerations, jointDynamics, feetConstraints, ... 
                                          ROBOT_DOF, reg, CONFIG)
                                          
% Generate the reference accelerations for both the floating base and the joints through a stack of task inverse kinematics structure

% setup parameters
PINV_TOL          = reg.pinvDampVb;     
    
%% Stack-of-tasks inverse kinematics

if CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION
    %Primary task: respect task constraints
    CoMrootAcc = taskAccelerations(1:6);
    JCoMroot   = Jtask(1:6, :);
    dnuCoMroot = pinv(JCoMroot, PINV_TOL) * (CoMrootAcc - dJnuDes(1:6));
    nullCoMroot= eye(6 + ROBOT_DOF) - pinv(JCoMroot, PINV_TOL) * JCoMroot;
    
    %Secondary task: feet contact constraints
    feetAcc     = zeros(12,1);
    Jfeet       = [Jtask(7:12, :) * feetConstraints(1);
                   Jtask(13:18, :) * feetConstraints(2)];
    dJnu_feet   = [dJnuDes(7:12, :) * feetConstraints(1);
                   dJnuDes(13:18, :) * feetConstraints(2)];
    dnu_feet    = pinv(Jfeet, PINV_TOL) * (feetAcc - dJnu_feet);
    null_feet   = eye(6 + ROBOT_DOF) - pinv(Jfeet, PINV_TOL) * Jfeet;
    
    %Tertiary task: joint posture
    S           = [zeros(ROBOT_DOF, 6) eye(ROBOT_DOF)];
    dnuPosture  = pinv(null_feet, PINV_TOL) * (pinv(S * nullCoMroot, PINV_TOL) * (jointDynamics - S * dnuCoMroot) - dnu_feet);
    
    dnuDes = dnuCoMroot + nullCoMroot * (dnu_feet + null_feet * dnuPosture);
    
else
    %Primary task: respect task constraints
    CoMrootAcc = taskAccelerations(1:6);
    JCoMroot   = Jtask(1:6, :);
    dnuCoMroot = pinv(JCoMroot, PINV_TOL) * (CoMrootAcc - dJnuDes(1:6));
    nullCoMroot= eye(6 + ROBOT_DOF) - pinv(JCoMroot, PINV_TOL) * JCoMroot;
    
    %Secondary task: feet contact constraints
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

end