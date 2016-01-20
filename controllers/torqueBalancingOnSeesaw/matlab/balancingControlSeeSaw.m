function [ tau ] = balancingControl(t, x,J_CoM,H,controlParams, model, robotWBM,regs)
%BALANCINGCONTROL Summary of this function goes here
%   Detailed explanation goes here

robotDoFs = model.robot.dofs;
% assert(robotDoFs <= 25)

[ seesaw_position, seesaw_vel, robotPos, robotVel] = state_partitioning(x, model.robot.dofs);

w_R_s          = rotationFromQuaternion(seesaw_position(1:4));
omega_w        = seesaw_vel(1:3);
g_w            = [0; 0; -9.81];
    
JdotNu         = robotWBM.JdotNu;
J              = robotWBM.J; 
Mass           = robotWBM.Mass;
genBiasForces  = robotWBM.genBiasForces;

robotCoM       = robotWBM.fwdkin.com;
robotLeftFoot  = robotWBM.fwdkin.l_sole;
robotRightFoot = robotWBM.fwdkin.r_sole;

selector       = [zeros(6, robotDoFs); eye(robotDoFs)];

Lambda         = J / Mass * selector;

% LambdaPinv     = pinv(Lambda, regs.pinvTol);
LambdaPinv     = Lambda'/(Lambda*Lambda' + regs.pinvDamp*eye(size(Lambda,1)));
NullLambda     = LambdaPinv * Lambda;
NullLambda     = eye(size(NullLambda)) - NullLambda;
seesaw         = model.seesaw;

e1             = [1;0;0];
e2             = [0;1;0];
e3             = [0;0;1];

r_w            = seesaw.delta * w_R_s * e3 - seesaw.rho * e3 ; %vector between com and contact point in world frame
s_R_w          = w_R_s';
g_s            = s_R_w * g_w; %gravity in the seesaw frame
r_s            = s_R_w * r_w; %r_w in seesaw frame
omega_s        = s_R_w * omega_w; %omega in seesaw frame
dr_s           = seesaw.rho * Sf(omega_s) * s_R_w * e3; %derivative of r_s

s_s_l          = [0; model.robot.lFootCentreDistance; model.seesaw.top];
s_s_r          = [0; model.robot.rFootCentreDistance; model.seesaw.top];

w_s_l          = w_R_s * s_s_l;
w_s_r          = w_R_s * s_s_r;

As             = [ eye(3), zeros(3), eye(3), zeros(3); ...
                   Sf(w_s_l), eye(3), Sf(w_s_r), eye(3)];

Delta          = [Sf(r_s - s_s_l); eye(3); Sf(r_s - s_s_r); eye(3)];
DeltaDot       = [eye(3); zeros(3); eye(3); zeros(3)] * Sf(dr_s);
xDcom          = J_CoM(1:3,:)*robotVel;

xDDcomStar     = controlParams.references.DDxcomDes + ...
                 controlParams.gains.xcomPGain * (controlParams.references.xcomDes(1:3)  - robotCoM(1:3)) + ...
                 controlParams.gains.xcomDGain * (controlParams.references.DxcomDes(1:3) - xDcom);
                   
Hdot_desired   = [ Mass(1,1)*xDDcomStar; 
                   -controlParams.gains.Hw*H(4:6)];
               
CentroidalMat  = [eye(3), zeros(3), eye(3), zeros(3);
                  Sf(robotLeftFoot-robotCoM), eye(3), Sf(robotRightFoot-robotCoM), eye(3)];
              
if seesaw.kind == 1 %Spherical seesaw
    Theta      = eye(3) + Sf(r_s) * seesaw.iota* Sf(r_s)';
    Iota_r     = eye(3) + seesaw.iota*norm(r_s)^2;

    Omega_0    = seesaw.iota + (1/det(Theta))*seesaw.iota*Sf(r_s)*Iota_r*Sf(r_s)*seesaw.iota;

    Omega_1    = (1/det(Theta))*seesaw.iota*Theta*Sf(r_s)*(Sf(dr_s) * omega_s - Sf(omega_s)^2 * r_s - g_s) ...
                     -Omega_0*Sf(omega_s)*seesaw.invIota*omega_s;

    Omega_2    = [-seesaw.iota*Theta*Sf(r_s)/det(Theta),Omega_0]*1/seesaw.mass;
    A          = [CentroidalMat];
    CentroidalRhs  = Hdot_desired - [Mass(1,1)*g_w; zeros(3,1)];
else if seesaw.kind == 2 %Semicylindrical seesaw
        lambda1= seesaw.delta*seesaw.iota(1,1)*(seesaw.rho*omega_s(1)^2 + 9.81)/(1 + seesaw.iota(1,1)*norm(r_s)^2);

        df     = seesaw.delta*w_R_s*e2 - seesaw.rho*e2;
        lambda2= seesaw.iota(1,1)/(seesaw.mass*(1 + seesaw.iota(1,1)*norm(r_s)^2));

        Omega_1= - e1*lambda1*w_R_s(3,2);

        Omega_2=  e1*lambda2*[df',e1']*blkdiag(w_R_s,w_R_s);
        A      = [  CentroidalMat];
%                    -lambda2*[df',e1']*  As ];
        CentroidalRhs  = [ Hdot_desired - [Mass(1,1)*g_w; zeros(3,1)]];
%                           -controlParams.gains.omegaGain * omega_s(1) + lambda1*w_R_s(3,2)];
    else
        Omega_1= zeros(3);
        Omega_2= zeros(3);        
        A      = [  CentroidalMat];
%                    -lambda2*[df',e1']*  As ];
        CentroidalRhs  = [ Hdot_desired - [Mass(1,1)*g_w; zeros(3,1)]];
    end
end





pinvA          = pinv(A);%A'/(A*A' + regs.pinvDamp*eye(size(A,1)));
NA             = pinvA * A;
NA             = eye(size(NA)) - NA;


             
f_desired      = pinvA* CentroidalRhs;


F              = blkdiag(w_R_s, w_R_s, w_R_s, w_R_s)*Delta* Omega_2* blkdiag(s_R_w,s_R_w) * As + J / Mass * J';

q              = robotPos(8:end);
qref           = controlParams.references.qDes;

hjBar          = genBiasForces(7:end) - controlParams.gains.posturalProp * (q - qref) - controlParams.gains.posturalDamp * robotVel(7:end) ...
                 - Mass(7:robotDoFs+6,1:6)/Mass(1:6,1:6)*genBiasForces(1:6);
JjBar          = J(:,7:end)'-Mass(7:robotDoFs+6,1:6)/Mass(1:6,1:6)* J(:,1:6)';
                 
tauModel       = LambdaPinv * (J / Mass * genBiasForces ...
               + blkdiag(w_R_s, w_R_s, w_R_s, w_R_s) * (Delta * Omega_1 ...
               + DeltaDot * omega_s + blkdiag(Sf(omega_s),Sf(omega_s),Sf(omega_s),Sf(omega_s)) * Delta * omega_s) ...
               - JdotNu) + NullLambda*hjBar;

Sigma          = -LambdaPinv *F - NullLambda*JjBar;
SigmaNA        = Sigma*NA; 
pinvSigmaNA    = SigmaNA'/(SigmaNA*SigmaNA' + regs.pinvDamp*eye(size(SigmaNA,1)));

f0             = -pinvSigmaNA*(tauModel + Sigma*f_desired);

tau            = tauModel + Sigma*f_desired + SigmaNA*f0;

end

