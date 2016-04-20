function [fNoQp,f_HDot,NA,tauModel,Sigmaf_HDot,SigmaNA,...
          HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet] = ...
          balancingControlSeeSaw(t, x,ConstraintsMatrix,bVectorConstraints,J_CoM,H,controlParams,...
                                 model, robot,reg,CONFIG,gain,ROBOT_DOF)
    %BALANCINGCONTROL Summary of this function goes here
    %   Detailed explanation goes here

    robotDoFs      = size(ROBOT_DOF,1);
    fNoQp          = zeros(12,1);
    tauModel       = zeros(robotDoFs,1);
    Sigmaf_HDot    = zeros(robotDoFs,1);
    SigmaNA        = zeros(robotDoFs,12);
    
    [ seesaw_pose, seesaw_vel, robotPos, robotVel] = state_partitioning(x, robotDoFs);

    w_R_s          = rotationFromQuaternion(seesaw_pose(1:4));
    s_omega_s      = seesaw_vel(1:3);
    gravityAcc     = [0; 0; -9.81];

    JdotNu         = robot.JdotNu;
    J              = robot.J; 
    M              = robot.M;
    genBiasForces  = robot.genBiasForces;
    
    gravityWrench  = [M(1,1)*gravityAcc; zeros(3,1)];

    w_p_com        = robot.fwdkin.w_p_com;
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

    r_w            = seesaw.delta * w_R_s * e3 - seesaw.rho * e3 ; %vector between com and contact point in world frame
    s_R_w          = w_R_s';
    g_s            = s_R_w * gravityAcc; %gravity in the seesaw frame
    r_s            = s_R_w * r_w; %r_w in seesaw frame
    % omega_s        = s_R_w * omega_w; %omega in seesaw frame
    dr_s           = seesaw.rho * Sf(s_omega_s) * s_R_w * e3; %derivative of r_s

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
                     controlParams.gains.xcomPGain * (controlParams.references.xcomDes(1:3)  - w_p_com(1:3)) + ...
                     controlParams.gains.xcomDGain * (controlParams.references.DxcomDes(1:3) - xDcom);

    Hdot_desired   = [ M(1,1)*xDDcomStar; 
                       -controlParams.gains.Hw*H(4:6)];

    CentroidalMat  = [eye(3), zeros(3), eye(3), zeros(3);
                      Sf(w_p_l_sole-w_p_com), eye(3), Sf(w_p_r_sole-w_p_com), eye(3)];
              

    Omega_1        = zeros(3);
    Omega_2        = zeros(3);        
    A              =  CentroidalMat;
    
    pinvA          = pinv(A);
    NA             = pinvA * A;
    NA             = eye(size(NA)) - NA;

    q              = robotPos(8:end);
    qref           = controlParams.references.qDes;
              
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

%     if CONFIG.CONTROLKIND == 1
        f_HDot         = pinvA* (Hdot_desired - gravityWrench);

        F              = blkdiag(w_R_s, w_R_s, w_R_s, w_R_s)*Delta* Omega_2* blkdiag(s_R_w,s_R_w) * As + J / M * J';

        hjBar          = genBiasForces(7:end) - M(7:robotDoFs+6,1:6)/M(1:6,1:6)*genBiasForces(1:6) ...
                         - controlParams.gains.posturalProp * (q - qref) - controlParams.gains.posturalDamp * robotVel(7:end);
                     
        JjBar          = J(:,7:end)'-M(7:robotDoFs+6,1:6)/M(1:6,1:6)* J(:,1:6)';

        tauModel       = LambdaPinv * (J / M * genBiasForces ...
                       + blkdiag(w_R_s, w_R_s, w_R_s, w_R_s) * (Delta * Omega_1 ...
                       + DeltaDot * s_omega_s + blkdiag(Sf(s_omega_s),Sf(s_omega_s),Sf(s_omega_s),Sf(s_omega_s)) * Delta * s_omega_s) ...
                       - JdotNu) + NullLambda*hjBar;

        Sigma          = -LambdaPinv *F - NullLambda*JjBar;
        SigmaNA        = Sigma*NA; 
        pinvSigmaNA    = pinvDamped(SigmaNA,reg.pinvDamp*1e-4);

        f0             = -pinvSigmaNA*(tauModel + Sigma*f_HDot);

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

   
%     end
end


