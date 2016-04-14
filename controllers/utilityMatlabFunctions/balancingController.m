function [tauModel,Sigma,NA,f_HDot, ...
              HessianMatrixQP1Foot,gradientQP1Foot,ConstraintsMatrixQP1Foot,bVectorConstraintsQp1Foot,...
              HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet,...
              errorCoM,qTilde,f]    =  ...
              balancingController(constraints,ROBOT_DOF_FOR_SIMULINK,ConstraintsMatrix,bVectorConstraints,...
              q,qDes,v, M, h , H,intHw,w_H_l_sole, w_H_r_sole, JL,JR, dJLv,dJRv, xcom,J_CoM, desired_x_dx_ddx_CoM,...
              impedances,intErrorCoM,ki_int_qtilde,reg,gain)
%BALANCING CONTROLLER
    %#codegen
    pos_leftFoot   = w_H_l_sole(1:3,4);
    w_R_l_sole     = w_H_l_sole(1:3,1:3);

    pos_rightFoot   = w_H_r_sole(1:3,4);
    w_R_r_sole      = w_H_r_sole(1:3,1:3);

    gainsPCOM       = gain.PCOM;
    gainsICOM       = gain.ICOM;
    gainsDCOM       = gain.DCOM;
    dampings        = gain.dampings;

    ROBOT_DOF       = size(ROBOT_DOF_FOR_SIMULINK,1);

    gravAcc         = 9.81;
    m               = M(1,1);
    Mb              = M(1:6,1:6);
    Mbj             = M(1:6,7:end);
    % Mj              = M(7:end,7:end);

    St              = [zeros(6,ROBOT_DOF);
                       eye(ROBOT_DOF,ROBOT_DOF)];
    grav            = [ zeros(2,1);
                       -m*gravAcc;
                        zeros(3,1)];

    xDcom           = J_CoM(1:3,:)*v;
    qD              = v(7:end);
    xDDcomStar      = desired_x_dx_ddx_CoM(:,3) - gainsPCOM*(xcom - desired_x_dx_ddx_CoM(:,1)) - gainsICOM*intErrorCoM - gainsDCOM*(xDcom - desired_x_dx_ddx_CoM(:,2));
    Pr              = pos_rightFoot - xcom; % Application point of the contact force on the right foot w.r.t. CoM
    Pl              = pos_leftFoot  - xcom; % Application point of the contact force on the left foot w.r.t. CoM


    AL              = [ eye(3),zeros(3);
                        Sf(Pl),  eye(3)];
    AR              = [ eye(3), zeros(3);
                        Sf(Pr), eye(3) ];

    A               = [ AL, AR];                  % dot(H) = mg + A*f
    pinvA           = pinv( A, reg.pinvTol)*constraints(1)*constraints(2)  ...
                    + [inv(AL);zeros(6)]*constraints(1)*(1-constraints(2)) ... 
                    + [zeros(6);inv(AR)]*constraints(2)*(1-constraints(1)); 
    NA              = (eye(12,12)-pinvA*A)*constraints(1)*constraints(2);

    Jc              = [ JL*constraints(1)  ;      % Time varying contact jacobian
                        JR*constraints(2) ];
    JcDv            = [dJLv*constraints(1) ;      % Time varying dot(J)*nu
                       dJRv*constraints(2)];

    JcMinv          = Jc/M;
    JcMinvSt        = JcMinv*St;
    JcMinvJct       = JcMinv*transpose(Jc);
    JBar            = transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6)); % multiplier of f in tau0

    qTilde          =  q-qDes;
    
    PInv_JcMinvSt   = pinvDamped(JcMinvSt,reg.pinvDamp); 
    NL              = eye(ROBOT_DOF) - PInv_JcMinvSt*JcMinvSt;

    
    Mbar            = M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end);

    NLMbar          = NL*Mbar;
    
    impedances      = diag(impedances)*pinv(NLMbar,reg.pinvTol) + reg.impedances*eye(ROBOT_DOF);
    dampings        = diag(dampings)*pinv(NLMbar,reg.pinvTol)   + reg.dampings*eye(ROBOT_DOF); 

   
    % Update constraints matrices 
    [constraintMatrixLeftFoot,constraintMatrixRightFoot] = ... 
        updateConstraintMatrices(ConstraintsMatrix,w_R_l_sole,w_R_r_sole,gain.footSize);
    
    %% QP PARAMETERS FOR TWO FEET STANDING
    % In the case the robot stands on two feet, the control objective is 
    % the minimization of the joint torques through the redundancy of the 
    % contact forces. By direct calculations one shows that the joint
    % torqes take the following form:
    %
    % tau = tauModel + Sigma*f_HDot + SigmaNA*f0
    %
    % where f0 is the redundancy of the contact wrenches. Then, the problem
    % is defined as follows:
    %
    % 1) f0  = argmin |tau(f0)|^2
    %          s.t.
    %          ConstraintsMatrixQP2Feet*f0 < bVectorConstraintsQp2Feet
    
    tauModel        = PInv_JcMinvSt*(JcMinv*h - JcDv) + NL*(h(7:end) - Mbj'/Mb*h(1:6) ...
                       -impedances*NLMbar*qTilde  -ki_int_qtilde -dampings*NLMbar*qD);
    
    Sigma           = -(PInv_JcMinvSt*JcMinvJct + NL*JBar);
    
    % Desired rate-of-change of the robot momentum
    HDotDes         = [ m*xDDcomStar ;
                        -gain.DAngularMomentum*H(4:end)-gain.PAngularMomentum*intHw]; 

    % Contact wrenches realizing the desired rate-of-change of the robot
    % momentum HDotDes when standing on two feet. Note that f_HDot is
    % different from zero only when both foot are in contact. 
    f_HDot          = pinvA*(HDotDes - grav)*constraints(1)*constraints(2);
    
    SigmaNA         = Sigma*NA;
  
    ConstraintsMatrix2Feet    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
    bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
    
    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
    bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

    HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*reg.HessianQP;
    gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);

    %% QP PARAMETERS FOR ONE FOOT STANDING
    % In the case the robot stands on one foot, there is no redundancy of
    % the contact wrenches. Hence, we cannot use this redundancy for
    % minimizing the joint torques. For this reason, the minimization
    % problem is modified as follows:
    %
    % 2) f = argmin|dot(H)(f) - dot(H)_des|^2
    %        s.t.
    %        ConstraintsMatrixQP1Foot*f < bVectorConstraintsQp1Foot
    %
    % where f is the contact wrench either of the left or on the right
    % foot.
    
    ConstraintsMatrixQP1Foot  = constraints(1) * (1 - constraints(2)) * constraintMatrixLeftFoot + ...
                                constraints(2) * (1 - constraints(1)) * constraintMatrixRightFoot;
    bVectorConstraintsQp1Foot = bVectorConstraints;

    A1Foot                    = AL*constraints(1)*(1-constraints(2)) + AR*constraints(2)*(1-constraints(1));
    HessianMatrixQP1Foot      = A1Foot'*A1Foot + eye(size(A1Foot,2))*reg.HessianQP;
    gradientQP1Foot           = -A1Foot'*(HDotDes - grav);

    %% DEBUG DIAGNOSTICS
    % Unconstrained solution for the problem 1)
    f0                        = -pinv(SigmaNA, reg.pinvTol)*(tauModel + Sigma*f_HDot);
    % Unconstrained contact wrenches
    f                         = f_HDot + NA*f0; 
    % Error on the center of mass
    errorCoM                  = xcom - desired_x_dx_ddx_CoM(:,1);
end

