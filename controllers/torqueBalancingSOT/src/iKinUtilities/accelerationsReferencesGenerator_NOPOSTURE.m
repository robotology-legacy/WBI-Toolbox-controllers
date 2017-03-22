function dnuDes   = accelerationsReferencesGenerator_NOPOSTURE(JFeet,JCoM,JPosture,dJFeetNu,dJCoMNu,feetDynamics,CoMDynamics,jointsDynamics)

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
dnu_joint  = 0*(pinv(JPosture*NullFeet*NullCoM, pinv_toll)*(jointsDynamics -JPosture*dnu_feet -JPosture*NullFeet*dnu_com));

%% Desired floating base and joints reference accelerations
dnuDes     = dnu_feet + NullFeet*(dnu_com + NullCoM*dnu_joint);
end


% % function dnuDes   = accelerationsReferencesGenerator_NOPOSTURE(JFeet,JCoM,JPosture,dJFeetNu,dJCoMNu,feetDynamics,CoMDynamics,jointsDynamics, ROBOT_DOF, reg)
% % 
% % % setup parameters
% % 
% % 
% % % null space projectors for primary and secondary task
% % NullCoM   = eye(6+ROBOT_DOF) - pinvDamped(JCoM, reg.pinvDampVb)*JCoM; 
% % % NullFeet   = eye(6+ROBOT_DOF) - pinvDamped(JFeet, reg.pinvDampVb)*JFeet;
% % NullFeet    = eye(6+ROBOT_DOF) - pinvDamped(JFeet*NullCoM, reg.pinvDampVb)*JFeet*NullCoM;
% % % NullCoM    = eye(6+ROBOT_DOF) - pinvDamped(JCoM*NullFeet, reg.pinvDampVb)*JCoM*NullFeet;
% % 
% % % stack of tasks inverse kinematics
% % 
% % %Primary task: achieve a desired CoM dynamics 
% % dnu_CoM   = pinvDamped(JCoM,reg.pinvDampVb)*(CoMDynamics -dJCoMNu);
% % % dnu_feet   = pinvDamped(JFeet,reg.pinvDampVb)*(feetDynamics -dJFeetNu);
% % 
% % % secondary task: respect the constraints at feet
% % dnu_feet    = pinvDamped(JFeet*NullCoM, reg.pinvDampVb)*(feetDynamics -dJFeetNu -JFeet*dnu_CoM);
% % % dnu_com    = pinvDamped(JCoM*NullFeet, reg.pinvDampVb)*(CoMDynamics -dJCoMNu -JCoM*dnu_feet);
% % 
% % % third task: achieve a desired joints position
% % % dnu_joint  = 0*(pinvDamped(JPosture*NullFeet*NullCoM, reg.pinvDampVb)*(jointsDynamics -JPosture*dnu_feet -JPosture*NullFeet*dnu_com));
% % dnu_joint  = 0*(pinvDamped(JPosture*NullCoM*NullFeet, reg.pinvDampVb)*(jointsDynamics -JPosture*dnu_CoM -JPosture*NullCoM*dnu_feet));
% % 
% % 
% % %% Desired floating base and joints reference accelerations
% % dnuDes     = dnu_CoM + NullCoM*(dnu_feet + NullFeet*dnu_joint);
% % end