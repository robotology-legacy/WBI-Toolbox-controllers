function  [KpMom,KpNull,KdMom,KdNull,KS,KD,KSopt,KDopt,ANull,BNull,Ax,Bx,KSdes,KDdes]...
          = linearizeAndTuning(xCoM,M,JG,Jc,w_H_l_sole,w_H_r_sole,LEFT_RIGHT_FOOT_IN_CONTACT,gain,reg,ROBOT_DOF_FOR_SIMULINK)
%LINEARIZATION AND GAIN TUNING

% ------------Initialization----------------
%% Config parameters
ndof               = size(ROBOT_DOF_FOR_SIMULINK,1);

%% Dynamics parameters
% mass matrix
Mb                 = M(1:6,1:6);
Mbj                = M(1:6,7:end);
Mjb                = M(7:end,1:6);
Mj                 = M(7:end,7:end);
Mbar               = Mj - Mjb/Mb*Mbj;
invMbar            = eye(ndof)/Mbar;

% contact jacobian
Jb                 = Jc(:,1:6);
Jj                 = Jc(:,7:end);

%% Forward kinematics parameters
% feet position
pos_leftFoot       = w_H_l_sole(1:3,4);
pos_rightFoot      = w_H_r_sole(1:3,4);

if sum(LEFT_RIGHT_FOOT_IN_CONTACT) ==1

    if LEFT_RIGHT_FOOT_IN_CONTACT(1) ==1
        
    Pf     = pos_leftFoot  - xCoM;            
    else
    Pf     = pos_rightFoot - xCoM;       
    end
% first task projector    
A          = [ eye(3),  zeros(3);
               skew(Pf),  eye(3)];    
pinvA      = eye(6)/A;

else
Pr         = pos_leftFoot  - xCoM;  
Pl         = pos_rightFoot - xCoM;

AL         = [ eye(3),  zeros(3);
               skew(Pl),  eye(3)];
AR         = [ eye(3),  zeros(3);
              skew(Pr),  eye(3)]; 
             
A          = [AL, AR];
pinvA      = pinv(A,reg.pinvTol);
end
    
%% Initial gains 
impedances         = diag(gain.impedances); 
dampings           = diag(gain.dampings);
intMomentumGains   = [gain.PCOM zeros(3); zeros(3) gain.PAngularMomentum*eye(3)];
MomentumGains      = [gain.DCOM zeros(3); zeros(3) gain.DAngularMomentum*eye(3)];
KSdes              = gain.KSdes;
KDdes              = gain.KDdes;

%% LINEARIZED JOINT SPACE DYNAMICS
Lambda             =  (Jj - Jb/Mb*Mbj)*invMbar;
MultFirstTask      =  Jb/Mb*transpose(Jb)*pinvA;
pinvLambda         =  pinvDamped(Lambda,reg.pinvDamp);
% pinvLambda       =  pinv(Lambda,reg.pinvTol);
NullLambda         =  eye(ndof) - pinvLambda*Lambda;

% contact jacobian
if LEFT_RIGHT_FOOT_IN_CONTACT(1) == 1
JGred              =  JG(:,7:end)-JG(:,1:6)*(eye(6)/Jb(1:6,1:6))*Jj(1:6,:);
else
JGred              =  JG(:,7:end)-JG(:,1:6)*(eye(6)/Jb(7:end,1:6))*Jj(7:end,:); 
end
posturalCorr       =  eye(ndof);

KS                 =  invMbar*(-pinvLambda*MultFirstTask*intMomentumGains*JGred + NullLambda*impedances*posturalCorr);
KD                 =  invMbar*(-pinvLambda*MultFirstTask*MomentumGains*JGred    + NullLambda*dampings*posturalCorr);

%% Kronecher Product and vectorization
Ax    = -invMbar*pinvLambda*MultFirstTask;
Bx    =  JGred;
ANull =  invMbar*NullLambda;
BNull =  posturalCorr;

if sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2   
Jc_base1    = Jc(1:6,1:6);
Jc_base2    = Jc(7:end,1:6);
Jc_joint1   = Jc(1:6,7:end);
Jc_joint2   = Jc(7:end,7:end);

AA     = (-Jc_base2*Jc_base1\Jc_joint1+Jc_joint2);
pinvAA = pinv(AA,1e-5);
NullAA = eye(ndof)-pinvAA*AA;

KSdes= NullAA*KSdes;
KDdes= NullAA*KDdes;
end

CONFIG.ndof           = ROBOT_DOF_FOR_SIMULINK;
CONFIG.pinv_tol       = reg.pinvTol;
CONFIG.pinv_damp      = reg.pinvDamp;
CONFIG.feet_on_ground = LEFT_RIGHT_FOOT_IN_CONTACT; 

[KpMom,KpNull] = kronVectorization(Ax,Bx,ANull,BNull,KSdes,CONFIG);
[KdMom,KdNull] = kronVectorization(Ax,Bx,ANull,BNull,KDdes,CONFIG);

%% Optimized KS and KD
KSopt = Ax*KpMom*Bx+ANull*KpNull*BNull;
KDopt = Ax*KdMom*Bx+ANull*KdNull*BNull;

%% DEBUG LINEARIZATION AND TUNING   
% Astate    = [zeros(ndof) eye(ndof); 
%               -KS           -KD];
%               
% AstateOpt = [zeros(ndof) eye(ndof); 
%               -KSopt      -KDopt];  
%           
% AstateDes = [zeros(ndof) eye(ndof); 
%               -KSdes      -KDdes]; 
%           
% eigAstate    = eig(Astate);          
% eigAstateOpt = eig(AstateOpt);
% eigAstateDes = eig(AstateDes);
% 
% disp('Eigenvalues of the linearized state matrix; eigenvalues of the optimized matrix')
% disp([sort(eigAstate) sort(eigAstateOpt) sort(eigAstateDes)])

end
