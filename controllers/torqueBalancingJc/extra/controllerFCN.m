function tau   =  controllerFCN(constraints, ROBOT_DOF, M, h, Jc, JcDNu,impedances, dampings, qTilde,qTildeD, qDesDD,reg)
% controllerFCN 
% computes the desired control torques
% at joints. 
%PINV_TOL        = 1e-8;

Mj               = M(7:end,7:end);

S                = [ zeros(6,ROBOT_DOF);
                     eye(ROBOT_DOF,ROBOT_DOF)];

%% terms from the constraint equation
Jct              = transpose(Jc);
JcMinv           = Jc/M;
JcMinvJct        = JcMinv*transpose(Jc);

smoothReg        = [(1-constraints(1))*eye(6),zeros(6);
                    zeros(6),(1-constraints(2))*eye(6)]*reg.pinvDamp;
                
%% joints space controller 
LambdaJcMinv     =  (JcMinvJct + smoothReg)\JcMinv;

Nj       = S'*(eye(ROBOT_DOF+6) - Jct*LambdaJcMinv)*S;

invNj  = (Nj')/(Nj*Nj' + reg.pinvDamp*eye(size(Nj,1)));  
%pinvB_bar  = pinv(B_bar,PINV_TOL);
tauE     = S'*Jct*(LambdaJcMinv*h - (JcMinvJct + smoothReg)\JcDNu);
tau      = invNj*(Mj*qDesDD -diag(impedances)*qTilde -diag(dampings)*qTildeD -tauE);

end
