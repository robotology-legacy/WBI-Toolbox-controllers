function dnuDes   = accelerationsReferencesGenerator(JFeet,JCoM,JPosture,dJFeetNu,dJCoMNu,feetDynamics,CoMDynamics,jointsDynamics)

% setup parameters
ndof       = length(JCoM(1,7:end));
pinv_toll  = 5e-7;  

% null space projectors for primary and secondary task
NullFeet   = eye(6+ndof) - pinv(JFeet,pinv_toll)*JFeet;
NullCoM    = eye(6+ndof) - pinv(JCoM*NullFeet,pinv_toll)*JCoM*NullFeet;

% stack of task inverse kinematics. Primary task: respect the constraints
% at feet
dnu_feet   = pinv(JFeet,pinv_toll)*(feetDynamics -dJFeetNu);

% secondary task: achieve a desired CoM dynamics
dnu_com    = pinv(JCoM*NullFeet, pinv_toll)*(CoMDynamics -dJCoMNu -JCoM*dnu_feet);

% third task: achieve a desired joints position
dnu_joint  = pinv(JPosture*NullFeet*NullCoM, pinv_toll)*(jointsDynamics -JPosture*dnu_feet -JPosture*NullFeet*dnu_com);

%% Desired floating base and joints reference accelerations
dnuDes     = dnu_feet + NullFeet*(dnu_com + NullCoM*dnu_joint);
end