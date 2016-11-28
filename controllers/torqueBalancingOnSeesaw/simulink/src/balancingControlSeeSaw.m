function [comError,fNoQp,f_HDot,NA,tauModel,Sigmaf_HDot,SigmaNA,...
          HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet, psat] = ...
          balancingControlSeeSaw(t, x,ConstraintsMatrix,bVectorConstraints,J_CoM,H,intHw,controlParams,...
                                 model, robot,reg,CONFIG,gain,ROBOT_DOF)
    %BALANCINGCONTROL Summary of this function goes here
    %   Detailed explanation goes here

    persistent w_p_com_total_0;

                   
    robotDoFs      = size(ROBOT_DOF,1);

    
    [ seesaw_pose, seesaw_vel, robotPos, robotVel] = state_partitioning(x, robotDoFs);

    q              = robotPos(8:end);
    qref           = controlParams.references.qDes;
    w_R_s          = rotationFromQuaternion(seesaw_pose(1:4));
    s_omega_s      = seesaw_vel(1:3);
    w_omega_s      = w_R_s*s_omega_s;
    gravityAcc     = [0; 0; -9.81];

    JdotNu         = robot.JdotNu;
    J              = robot.J; 
    M              = robot.M;
    mR             = M(1,1);
    mS             = model.seesaw.mass;
    mT             = mR + mS;
    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,  Mbj
    %         Mbj',Mj  ];  Mb \in R^{6x6}, Mbj \in R^{6x6+nDof}, Mj \in R^{nDofxnDof}
    Mb              = M(1:6,1:6);
    Mbj             = M(1:6,7:end);
    Mj              = M(7:end,7:end);

    St              = [zeros(6,robotDoFs);
                       eye(robotDoFs,robotDoFs)];
  
                   
    genBiasForces  = robot.genBiasForces;
    
    gravityWrench  = [M(1,1)*gravityAcc; zeros(3,1)];

    w_p_com_R      = robot.fwdkin.w_p_com;
    w_p_l_sole     = robot.fwdkin.w_H_l_sole(1:3,4);
    w_p_r_sole     = robot.fwdkin.w_H_r_sole(1:3,4);
    w_R_l_sole     = robot.fwdkin.w_H_l_sole(1:3,1:3);
    w_R_r_sole     = robot.fwdkin.w_H_l_sole(1:3,1:3);

    selector       = [zeros(6, robotDoFs); eye(robotDoFs)];

    Lambda         = J / M * selector;

    LambdaPinv     = pinvDamped(Lambda,reg.pinvDamp); 
    
    NullLambda     = LambdaPinv * Lambda;
    NullLambda     = eye(size(NullLambda)) - NullLambda;
    
    seesaw         = model.seesaw;

    e1             = [1;0;0];
    e2             = [0;1;0];
    e3             = [0;0;1];
    
    %vector between com and contact point in world frame
    r_w            = seesaw.delta * w_R_s * e3 - seesaw.rho * e3 ; 
    s_R_w          = w_R_s';
    g_s            = s_R_w * gravityAcc; %gravity in the seesaw frame
    r_s            = s_R_w * r_w; %r_w in seesaw frame
    % omega_s        = s_R_w * omega_w; %omega in seesaw frame
    dr_s           = seesaw.rho * Sf(s_omega_s) * s_R_w * e3; %derivative of r_s

    s_s_l          = [0; model.robot.lFootDistanceCenter; model.seesaw.top];
    s_s_r          = [0; model.robot.rFootDistanceCenter; model.seesaw.top];

    w_s_l          = w_R_s * s_s_l;
    w_s_r          = w_R_s * s_s_r;

    
    w_p_com_s      = w_p_l_sole - w_s_l;
        
    As             = [ eye(3), zeros(3), eye(3), zeros(3); ...
                       Sf(w_s_l), eye(3), Sf(w_s_r), eye(3)];

    Delta          = [Sf(r_s - s_s_l); eye(3); Sf(r_s - s_s_r); eye(3)];
    DeltaDot       = [eye(3); zeros(3); eye(3); zeros(3)] * Sf(dr_s);
    w_v_com_R      = J_CoM(1:3,:)*robotVel;

    psat = saturate(controlParams.gain.PCOM * (controlParams.references.xcomDes(1:3)  - w_p_com_R(1:3)), -controlParams.gain.P_SATURATION, controlParams.gain.P_SATURATION);
    xDDcomStar     = controlParams.references.DDxcomDes + ...
                     psat + ...
                     controlParams.gain.DCOM * (controlParams.references.DxcomDes(1:3) - w_v_com_R);

    Hdot_desired   = [ mR*xDDcomStar; 
                       -controlParams.gain.DAngularMomentum*H(4:end)-controlParams.gain.PAngularMomentum*intHw];

    CentroidalMat  = [eye(3), zeros(3), eye(3), zeros(3);
                      Sf(w_p_l_sole-w_p_com_R), eye(3), Sf(w_p_r_sole-w_p_com_R), eye(3)];
              

    Omega_1        = zeros(3);
    Omega_2        = zeros(3);        
              
    lambda2        = 0;
    Omega_2Bar     = zeros(1,6);
    
    if seesaw.kind == 1 %Spherical seesaw
        Theta          = eye(3) + Sf(r_s) * seesaw.iota* Sf(r_s)';
        Iota_r         = eye(3) + seesaw.iota*norm(r_s)^2;

        
        Omega_0        = seesaw.iota + (1/det(Theta))*seesaw.iota*Sf(r_s)*Iota_r*Sf(r_s)*seesaw.iota;

        Omega_1        = (1/det(Theta))*seesaw.iota*Theta*Sf(r_s)*(Sf(dr_s) * s_omega_s - Sf(s_omega_s)^2 * r_s - g_s) ...
                         -Omega_0*Sf(s_omega_s)*seesaw.invIota*s_omega_s;

        Omega_2        = [-seesaw.iota*Theta*Sf(r_s)/det(Theta),Omega_0]*1/seesaw.mass;
        
    elseif seesaw.kind == 2 %Semicylindrical seesaw
            
        lambda1        = seesaw.delta*seesaw.iota(1,1)*(seesaw.rho*s_omega_s(1)^2 -gravityAcc(3))/(1 + seesaw.iota(1,1)*norm(r_s)^2);

        Omega_2Bar     = [(seesaw.delta*w_R_s*e2-seesaw.rho*e2)',e1'];
        lambda2        = seesaw.iota(1,1)/(seesaw.mass*(1 + seesaw.iota(1,1)*norm(r_s)^2));

        Omega_1        = - e1*lambda1*w_R_s(3,2);

        Omega_2        =  e1*lambda2*Omega_2Bar*blkdiag(w_R_s,w_R_s);
    end
    
    comError           = controlParams.references.xcomDes(1:3)  - w_p_com_R(1:3);


    if CONFIG.CONTROLKIND == 1
        A              =  CentroidalMat;

        pinvA          = pinv(A);
        NA             = pinvA * A;
        NA             = eye(size(NA)) - NA;
    
        f_HDot         = pinvA* (Hdot_desired - gravityWrench);

    elseif CONFIG.CONTROLKIND == 2

        A              =  [CentroidalMat;
                           -lambda2*Omega_2Bar*As];

%         pinvA          = pinvDamped(A,reg.pinvDampA);

        pinvA          = pinv(A);
        NA             = pinvA * A;
        NA             = eye(size(NA)) - NA;
        
        theta          = atan2(w_R_s(3,2),w_R_s(2,2))*180/pi;
        thetaDot       = w_omega_s(1)*180/pi;
        
        desiredDyn     = [ Hdot_desired 
                          -gain.seesawKP*theta-gain.seesawKD*thetaDot];
        
        f_HDot         = pinvA* (desiredDyn - [gravityWrench;0]);
        
    elseif CONFIG.CONTROLKIND == 4

        Ar              =  CentroidalMat;
        Aseesaw         = -lambda2*Omega_2Bar*As;

%         pinvAr          = pinvDamped(Ar,reg.pinvDampA);
%         pinvAs          = pinvDamped(As,reg.pinvDampA);

        pinvAr          = pinv(Ar,0.0001);
        pinvAseesaw     = pinv(Aseesaw,0.0001);

        NAr             = pinvAr * Ar;
        NAr             = eye(size(NAr)) - NAr;
        NAs             = pinvAseesaw * Aseesaw;
        NAs             = eye(size(NAs)) - NAs;

        NA              = NAr*NAs;
        
        theta           = atan2(w_R_s(3,2),w_R_s(2,2))*180/pi;
        thetaDot        = w_omega_s(1)*180/pi;
        
        desiredDyn_r    =  Hdot_desired; 
        desiredDyn_s    = -gain.seesawKP*theta-gain.seesawKD*thetaDot;
        
        f_HDot_r        = pinvAr* (desiredDyn_r - gravityWrench);
        f_HDot_s        = pinv(Aseesaw*NAr,0.0001)*(desiredDyn_s -Aseesaw*f_HDot_r);

        f_HDot          = f_HDot_r + NAr*f_HDot_s;

    elseif CONFIG.CONTROLKIND == 3
        
        w_p_com_total  =  (mS*w_p_com_s + mR*w_p_com_R)/(mR+mS);
        if isempty(w_p_com_total_0)
            w_p_com_total_0 = w_p_com_total;
        end
           
        w_v_com_s      = Sf(r_w)*w_omega_s;
        
        w_v_com_total  =  (mS*w_v_com_s + mR*w_v_com_R)/(mR+mS);

        Theta          = eye(3) + Sf(r_s) * seesaw.iota* Sf(r_s)';

        w_F_c1         =  (w_R_s/Theta)*(mS*Sf(dr_s)*s_omega_s -mS*g_s-mS*Sf(s_omega_s)^2*r_s);
        
        AL             =  -(w_R_s/Theta)*[-s_R_w,mS*Sf(r_s)*seesaw.iota*s_R_w]*As;
        
        
        A              =  [AL;
                           CentroidalMat(4:6,:)];

%         pinvA          = pinvDamped(A,reg.pinvDampA);
        pinvA          = pinv(A);


        NA             = pinvA * A;
        NA             = eye(size(NA)) - NA;
        
        desiredDyn     = Hdot_desired;
        desiredDyn(1:3) ...
                       = -mT*(controlParams.gain.PCOM * (w_p_com_total-w_p_com_total_0) ...
                             +controlParams.gain.DCOM *  w_v_com_total);
        
        f_HDot         = pinvA* (desiredDyn - [mT*gravityAcc+w_F_c1;zeros(3,1)]);
      
%         desDynError    = norm(desiredDyn - (A*f_HDot + [mT*gravityAcc+w_F_c1;zeros(3,1)]))
        
        comError       = w_p_com_total_0  - w_p_com_total;

    else
        fNoQp          = zeros(12,1);
        tauModel       = zeros(robotDoFs,1);
        Sigmaf_HDot    = zeros(robotDoFs,1);
        SigmaNA        = zeros(robotDoFs,12);
        NA             = zeros(12);
        Sigma          = zeros(robotDoFs,12);
        f_HDot         = zeros(12,1); 
    end
    F               = J / M * J' + ...
                      CONFIG.CONSIDERSEESAWDYN*blkdiag(w_R_s,w_R_s,w_R_s,w_R_s)*Delta*Omega_2* blkdiag(s_R_w,s_R_w)* As;

    
    JcMinv          = J/M;
    JcMinvSt        = JcMinv*St;

    
    PInv_JcMinvSt   = pinvDamped(JcMinvSt,reg.pinvDamp); 
    % nullJcMinvSt  = null space of PInv_JcMinvSt
    nullJcMinvSt    = eye(robotDoFs) - PInv_JcMinvSt*JcMinvSt;
    
    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    Mbar            = Mj-Mbj'/Mb*Mbj;

    NLMbar          = nullJcMinvSt*Mbar;
    
    % Adaptation of control gains for back compatibility with older
    % versions of the controller
    impedances      = controlParams.gain.posturalProp;%*pinv(NLMbar,reg.pinvTol) + reg.impedances*eye(robotDoFs);
    dampings        = controlParams.gain.posturalDamp;%*pinv(NLMbar,reg.pinvTol) + reg.dampings*eye(robotDoFs); 
    
    hjBar          = genBiasForces(7:end) - M(7:robotDoFs+6,1:6)/M(1:6,1:6)*genBiasForces(1:6) ...
                     -  impedances* (q - qref) -  dampings * robotVel(7:end);

    JjBar          = J(:,7:end)'-M(7:robotDoFs+6,1:6)/M(1:6,1:6)* J(:,1:6)';

    tauModel       = LambdaPinv * (J / M * genBiasForces - JdotNu ...
                   + CONFIG.CONSIDERSEESAWDYN*blkdiag(w_R_s, w_R_s, w_R_s, w_R_s) *(Delta * Omega_1 + DeltaDot * s_omega_s + blkdiag(Sf(s_omega_s),Sf(s_omega_s),Sf(s_omega_s),Sf(s_omega_s)) * Delta * s_omega_s)) ...
                   + NullLambda*hjBar;

    Sigma          = -LambdaPinv *F - NullLambda*JjBar;
    SigmaNA        = Sigma*NA; 

    f0             = -pinvDamped(SigmaNA,reg.pinvDamp*1e-4)*(tauModel + Sigma*f_HDot);

    Sigmaf_HDot    = Sigma*f_HDot;

    fNoQp          = f_HDot  + NA*f0;
        
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(w_R_l_sole',w_R_l_sole');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(w_R_r_sole',w_R_r_sole');
    ConstraintsMatrix2Feet    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
    bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];

    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
    bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

    HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*reg.HessianQP;
    gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);
    

end


