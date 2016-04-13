function [tauModel,Sigma,NA,fHdotDesC1C2, ...
              HessianMatrixQP1Foot,gradientQP1Foot,ConstraintsMatrixQP1Foot,bVectorConstraintsQp1Foot,...
              HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet,...
              errorCoM,qTilde,f]    =  ...
              balancingController(constraints,ROBOT_DOF_FOR_SIMULINK,ConstraintsMatrix,bVectorConstraints,...
              q,qDes,v, M, h , H,intHw,w_H_l_sole, w_H_r_sole, JL,JR, dJLv,dJRv, xcom,J_CoM, desired_x_dx_ddx_CoM,...
              impedances,intErrorCoM,ki_int_qtilde,reg,gain)
%BALANCING CONTROLLER
    %#codegen
    pos_leftFoot   = w_H_l_sole(1:3,4);
    rotMatLeftFoot = w_H_l_sole(1:3,1:3);

    pos_rightFoot   = w_H_r_sole(1:3,4);
    rotMatRightFoot = w_H_r_sole(1:3,1:3);

    footSize        = gain.footSize;
    gainsPCOM       = gain.PCOM;
    gainsICOM       = gain.ICOM;
    gainsDCOM       = gain.DCOM;
    dampings        = gain.dampings;

    e1              = [1;0;0];
    e2              = [0;1;0];
    e3              = [0;0;1]; 
    ROBOT_DOF       = size(ROBOT_DOF_FOR_SIMULINK,1);


    gravAcc         = 9.81;
    m               = M(1,1);
    Mb              = M(1:6,1:6);
    Mbj             = M(1:6,7:end);
    % Mj              = M(7:end,7:end);

    St              = [  zeros(6,ROBOT_DOF);
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

    A               = [ AL, AR];

    Jc              = [ JL*constraints(1) ;
                        JR*constraints(2)];
    JcDv            = [ dJLv*constraints(1) ;
                        dJRv*constraints(2)];

    JcMinv          = Jc/M;
    JcMinvSt        = JcMinv*St;
    JcMinvJct       = JcMinv*transpose(Jc);

    pinvA           = pinv( A, reg.pinvTol)*constraints(1)*constraints(2)  ...
                    + [inv(AL);zeros(6)]*constraints(1)*(1-constraints(2)) ... 
                    + [zeros(6);inv(AR)]*constraints(2)*(1-constraints(1)); 

    PInv_JcMinvSt   = pinvDamped(JcMinvSt,reg.pinvDamp); 

    NL              = eye(ROBOT_DOF) - PInv_JcMinvSt*JcMinvSt;
    Mbar            = M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end);

    NLMbar          = NL*Mbar;
    impedances      = diag(impedances)*pinv(NLMbar,reg.pinvTol) + reg.impedances*eye(ROBOT_DOF);
    dampings        = diag(dampings)*pinv(NLMbar,reg.pinvTol)   + reg.dampings*eye(ROBOT_DOF); 

    HDotDes         = [ m*xDDcomStar ;
                        -gain.DAngularMomentum*H(4:end)-gain.PAngularMomentum*intHw]; 

    f_HDot          = pinvA*(HDotDes - grav);

    fHdotDesC1C2    =  f_HDot*constraints(1)*constraints(2);

    NA              = (eye(12,12)-pinvA*A)*constraints(1)*constraints(2);
    JBar            = transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6)); % multiplier of f in tau0

    qTilde          =  q-qDes;

    Sigma           = -(PInv_JcMinvSt*JcMinvJct + NL*JBar);
    SigmaNA         = Sigma*NA;

    tauModel        = PInv_JcMinvSt*(JcMinv*h - JcDv) + NL*(h(7:end) - Mbj'/Mb*h(1:6) ...
                       -impedances*NLMbar*qTilde  -ki_int_qtilde -dampings*NLMbar*qD);

    CL              = ConstraintsMatrix; 
    CL(end-4,1:3)   = -e3'*rotMatLeftFoot;                
    CL(end-3,:)     = [ footSize(1,1)*e3'*rotMatLeftFoot', e2'*rotMatLeftFoot'];
    CL(end-2,:)     = [-footSize(1,2)*e3'*rotMatLeftFoot',-e2'*rotMatLeftFoot'];

    CL(end-1,:)     = [ footSize(2,1)*e3'*rotMatLeftFoot',-e1'*rotMatLeftFoot'];
    CL(end  ,:)     = [-footSize(2,2)*e3'*rotMatLeftFoot', e1'*rotMatLeftFoot'];

    CR              = ConstraintsMatrix;
    CR(end-4,1:3)   = -e3'*rotMatRightFoot;  
    CR(end-3,:)     = [ footSize(1,1)*e3'*rotMatRightFoot', e2'*rotMatRightFoot'];
    CR(end-2,:)     = [-footSize(1,2)*e3'*rotMatRightFoot',-e2'*rotMatRightFoot'];

    CR(end-1,:)     = [ footSize(2,1)*e3'*rotMatRightFoot',-e1'*rotMatRightFoot'];
    CR(end  ,:)     = [-footSize(2,2)*e3'*rotMatRightFoot', e1'*rotMatRightFoot'];


    ConstraintsMatrix2Feet    = blkdiag(CL,CR);
    bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
    
    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
    bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

    HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*reg.HessianQP;
    gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);

    
    ConstraintsMatrixQP1Foot  = constraints(1) * (1 - constraints(2)) * CL + ...
                               constraints(2) * (1 - constraints(1)) * CR;
    bVectorConstraintsQp1Foot = bVectorConstraints;

    A1Foot                    = AL*constraints(1)*(1-constraints(2)) + AR*constraints(2)*(1-constraints(1));
    HessianMatrixQP1Foot      = A1Foot'*A1Foot + eye(size(A1Foot,2))*reg.HessianQP;
    gradientQP1Foot           = -A1Foot'*(HDotDes - grav);

    f0                        = -pinv(SigmaNA, reg.pinvTol)*(tauModel + Sigma*f_HDot);

    f                         = f_HDot + NA*f0;

    errorCoM                  = xcom - desired_x_dx_ddx_CoM(:,1);
end

