function dnuDes   = referenceAcceleration(Jtask, dJnuDes, taskAccelerations, jointDynamics, ... 
                                                      constraints, ROBOT_DOF)
                                          
% Generate the reference accelerations for both the floating base and the joints through a stack of task inverse kinematics structure

% setup parameters
PINV_TOL          = 5e-7;

% Task corrections for foot in contact with ground
Jcontact          = [Jtask( 7:12,:) * constraints(1);
                     Jtask(13:18,:) * constraints(2)];
        
dJnuContact       = [dJnuDes( 7:12) * constraints(1); 
                     dJnuDes(13:18) * constraints(2)];  

Jtask             = [Jtask( 1:6, :);
                     Jtask( 7:12,:) * constraints(1);
                     Jtask(13:18,:) * constraints(2);
                     Jcontact]; 

dJnuDes           = [dJnuDes( 1:6 );
                     dJnuDes( 7:12) * constraints(1);
                     dJnuDes(13:18) * constraints(2);
                     dJnuContact];     
        
taskAccelerations = [taskAccelerations; 
                     zeros(12,1)];        
    
%% Stack-of-tasks inverse kinematics

%Primary task: respect task constraints and contact constraints
dnuTask    = pinv(Jtask, PINV_TOL) * (taskAccelerations - dJnuDes);
% null space projector for primary task
NullTask   = eye(6 + ROBOT_DOF) - pinv(Jtask, PINV_TOL)*Jtask;

%Secondary task: achieve desired joints position
S          = [zeros(ROBOT_DOF, 6) eye(ROBOT_DOF)];
dnuPosture = pinv(S*NullTask, PINV_TOL)*(jointDynamics - S*dnuTask);

%Reference accelerations for floating base and joints
dnuDes     = dnuTask + NullTask*dnuPosture;

end