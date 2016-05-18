function [tau,fc]   =  JointController(constraints, M, h, Jc, dJcNu,impedances, dampings, inverseKinRef, qj,Nu,reg)
%% Joint Error
qTilde           = qj - inverseKinRef(:,1);
dqTilde          = Nu(7:end) - inverseKinRef(:,2);
ddqDes           = inverseKinRef(:,3);

Mj               = M(7:end,7:end);
ROBOT_DOF        = size(Mj,1);

S                = [ zeros(6,ROBOT_DOF);
                     eye(ROBOT_DOF,ROBOT_DOF)];

%% Terms from the constraint equations
Jct              = transpose(Jc);
JcMinv           = Jc/M;
JcMinvJct        = JcMinv*transpose(Jc);

smoothReg        = [(constraints(1))*eye(6),zeros(6);
                    zeros(6),(constraints(2))*eye(6)]*reg.pinvDamp;
                
%% Joints space controller 
LambdaJcMinv     =  (JcMinvJct + smoothReg)\JcMinv;

Nj       = S'*(eye(ROBOT_DOF+6) - Jct*LambdaJcMinv)*S;

invNj    = (Nj')/(Nj*Nj' + reg.pinvTol*eye(size(Nj,1)));  

tauE     =  S'*Jct*(LambdaJcMinv*h - (JcMinvJct + smoothReg)\dJcNu);

tau      = invNj*(Mj*(ddqDes) -diag(impedances)*qTilde -diag(dampings)*dqTilde -tauE);

fc       = eye(size(Jc,1))/(JcMinvJct + smoothReg)*(JcMinv*h - JcMinv*S*tau -dJcNu);

end
