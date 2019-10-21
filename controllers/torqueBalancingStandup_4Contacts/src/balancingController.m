%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [tauModel,Sigma,f_HDot,NA, ...
          HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet, ...
          HessianMatrixQP1Foot,gradientQP1Foot,ConstraintsMatrixQP1Foot,bVectorConstraintsQp1Foot,...
          HessianMatrixQP2Feet2Legs,gradientQP2Feet2Legs,ConstraintsMatrixQP2Feet2Legs,bVectorConstraintsQp2Feet2Legs, ...          
          errorCoM,qTilde,f]  =  ...
          balancingController(constraints,ROBOT_DOF_FOR_SIMULINK,ConstraintsMatrix,bVectorConstraints,ConstraintsMatrixLegs,bVectorConstraintsLegs,...
                              q, qDes, v, M, h, H, intHw, feetPose, legsPose, JFeet, JLegs, dJFeet, dJLegs, xcom, J_CoM, desired_x_dx_ddx_CoM,...
                              gainsPCOM, gainsDCOM, impedances_old, intErrorCoM, ki_int_qtilde, reg, gain, legsInContact)                          
    % BALANCING CONTROLLER

    %% DEFINITION OF CONTROL AND DYNAMIC VARIABLES
    % feet position and orientation
    w_H_l_sole      = feetPose(:,1:4);
    w_H_r_sole      = feetPose(:,5:8);
    
    pos_leftFoot    = w_H_l_sole(1:3,4);
    w_R_l_sole      = w_H_l_sole(1:3,1:3);

    pos_rightFoot   = w_H_r_sole(1:3,4);
    w_R_r_sole      = w_H_r_sole(1:3,1:3);
    
    % legs position and orientation
    w_H_l_leg       = legsPose(:,1:4);
    w_H_r_leg       = legsPose(:,5:8);
    
    pos_leftLeg     = w_H_l_leg(1:3,4);
    w_R_l_leg       = w_H_l_leg(1:3,1:3);

    pos_rightLeg    = w_H_r_leg(1:3,4);
    w_R_r_leg       = w_H_r_leg(1:3,1:3);
    
    % gains matrices
    gainsICOM       = zeros(3,1);
    dampings_old    = gain.dampings;

    ROBOT_DOF       = size(ROBOT_DOF_FOR_SIMULINK,1);
    gravAcc         = 9.81;
    
    % Mass of the robot.
    m               = M(1,1);
    
    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,   Mbj
    %         Mbj', Mj ];  
    %   Mb \in R^{6x6}, Mbj \in R^{6x6+nDof}, Mj \in R^{nDofxnDof}
    %
    Mb              = M(1:6,1:6);
    Mbj             = M(1:6,7:end);
    Mj              = M(7:end,7:end);

    St              = [zeros(6,ROBOT_DOF);
                       eye(ROBOT_DOF,ROBOT_DOF)];
    gravityWrench   = [ zeros(2,1);
                       -m*gravAcc;
                        zeros(3,1)];

    % Velocity of the center of mass
    xDcom           = J_CoM(1:3,:)*v;
    
    % Joint velocity
    qD              = v(7:end);
    
    % Joint position error
    qTilde          =  q-qDes;
    
    % Desired acceleration for the center of mass
    xDDcomStar      = desired_x_dx_ddx_CoM(:,3) - gainsPCOM.*(xcom - desired_x_dx_ddx_CoM(:,1)) - gainsICOM.*intErrorCoM - gainsDCOM.*(xDcom - desired_x_dx_ddx_CoM(:,2));
   
    % Application point of the contact force on the right foot w.r.t. CoM
    Pr              = pos_rightFoot - xcom; 
    
    % Application point of the contact force on the left foot w.r.t. CoM
    Pl              = pos_leftFoot  - xcom; 
    
    % Application point of the contact force on the right leg w.r.t. CoM
    PrLeg           = pos_rightLeg  - xcom; 
    
    % Application point of the contact force on the left leg w.r.t. CoM
    PlLeg           = pos_leftLeg   - xcom; 

    % The following variables serve for determining the rate-of-change of
    % the robot's momentum. In particular, when balancing on two feet, one has:
    %
    %   dot(H) = gravityWrench +  AL*f_L + AR*f_R
    %          = gravityWrench + [AL,AR]*f
    %
    % where  f_L and f_R are the contact wrenches acting on the left and
    % right foot, respectively, and f = [f_L;f_R].
    
    AL              = [ eye(3),  zeros(3);
                        Sf(Pl),  eye(3)];
    AR              = [ eye(3),  zeros(3);
                        Sf(Pr),  eye(3) ];
    % It is assumed in the following that the legs must only be either both 
    % in contact or none of them. Only one leg in contact is not admissible.
    AL_leg          = [ eye(3),    zeros(3);
                        Sf(PlLeg), eye(3)];
    AR_leg          = [ eye(3),    zeros(3);
                        Sf(PrLeg), eye(3) ];

    A               = [AL, AR, AL_leg, AR_leg];                                                                                  % dot(H) = mg + A*f
    pinvA           = pinv(A, reg.pinvTol)*constraints(1)*constraints(2)*legsInContact ...                                       % all points in contact
                    + [pinv([AL,AR],reg.pinvTol);zeros(6);zeros(6)]*constraints(1)*constraints(2)*(1-legsInContact) ...          % only feet in contact
                    + [inv(AL);zeros(6);zeros(6);zeros(6)]*constraints(1)*(1-constraints(2))*(1-legsInContact) ...               % left foot in contact
                    + [zeros(6);inv(AR);zeros(6);zeros(6)]*constraints(2)*(1-constraints(1))*(1-legsInContact);                  % right foot in contact             
                
    % Null space of the matrix A            
    NA              = (eye(24,24)-pinvA*A)*constraints(1)*constraints(2)*legsInContact +...
                      [(eye(12,12)-pinv([AL,AR],reg.pinvTol)*([AL,AR])) zeros(12,12);...
                        zeros(12,12)                                    zeros(12,12)]*constraints(1)*constraints(2)*(1-legsInContact);

    % Time varying contact jacobian
    JL              = JFeet(1:6,:);
    JR              = JFeet(7:12,:);
    
    JL_leg          = JLegs(1:6,:);
    JR_leg          = JLegs(7:12,:);
    
    Jc              = [JL*constraints(1);      
                       JR*constraints(2); 
                       JL_leg*legsInContact;
                       JR_leg*legsInContact];
                  
    % Time varying dot(J)*nu
    dJLv            = dJFeet(1:6);
    dJRv            = dJFeet(7:12);
    
    dJL_leg         = dJLegs(1:6);
    dJR_leg         = dJLegs(7:12);
    
    JcDv            = [dJLv*constraints(1);      
                       dJRv*constraints(2);
                       dJL_leg*legsInContact;      
                       dJR_leg*legsInContact;];

    JcMinv          = Jc/M;
    JcMinvSt        = JcMinv*St;
    JcMinvJct       = JcMinv*transpose(Jc);
    JBar            = transpose(Jc(:,7:end)) -Mbj'/Mb*transpose(Jc(:,1:6)); % multiplier of f in tau0

    Pinv_JcMinvSt   = pinvDamped(JcMinvSt,reg.pinvDamp); 
   
    % nullJcMinvSt  = null space of PInv_JcMinvSt
    nullJcMinvSt    = eye(ROBOT_DOF) - Pinv_JcMinvSt*JcMinvSt;

    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    Mbar            = Mj-Mbj'/Mb*Mbj;
    NLMbar          = nullJcMinvSt*Mbar;
    
    % Adaptation of control gains for back compatibility with older
    % versions of the controller
    impedances      = diag(impedances_old)*pinv(NLMbar,reg.pinvTol) + reg.impedances*eye(ROBOT_DOF);
    dampings        = diag(dampings_old)*pinv(NLMbar,reg.pinvTol)   + reg.dampings*eye(ROBOT_DOF); 
  
    %% QP PARAMETERS FOR TWO FEET STANDING
    % In the case the robot stands on two feet, the control objective is 
    % the minimization of the joint torques through the redundancy of the 
    % contact forces. By direct calculations one shows that the joint
    % torqes take the following form:
    %
    % 0) tau = tauModel + Sigma*f_HDot + SigmaNA*f0
    %
    % where f0 is the redundancy of the contact wrenches. Then, the problem
    % is defined as follows:
    %
    % 1) f0  = argmin |tau(f0)|^2
    %          s.t.
    %          ConstraintsMatrixQP2Feet*f0 < bVectorConstraintsQp2Feet
    
    % Update constraint matrices. The constraint matrix for the inequality
    % constraints in the problem 1) is built up startin from the constraint
    % matrix associated with each single foot. More precisely, the contact
    % wrench associated with the left foot (resp. right foot) is subject to
    % the following constraint:
    %
    % constraintMatrixLeftFoot*l_sole_f_L < bVectorConstraints
    %
    % In this case, however, f_L is expressed w.r.t. the frame l_sole,
    % which is solidal to the left foot. Since the controller uses contact
    % wrenches expressed w.r.t. a frame whose orientation is that of the
    % inertial frame, we have to update the constraint matrix according to
    % the transformation w_R_l_sole, i.e.
    %
    % constraintMatrixLeftFoot = ConstraintsMatrix*w_R_l_sole
    %
    % The same hold for the right foot
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(w_R_l_sole',w_R_l_sole');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(w_R_r_sole',w_R_r_sole');
    
    ConstraintsMatrix2Feet    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
    bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
    
    % Overwrite parameters for QP 2 Feet. Actually, this is only for a
    % matter of matrix dimensions
    JcMinv_2feet          = Jc(1:12,:)/M;
    JcMinvSt_2feet        = JcMinv_2feet*St;
    JcMinvJct_2feet       = JcMinv_2feet*transpose(Jc(1:12,:));
    JBar_2feet            = transpose(Jc(1:12,7:end)) -Mbj'/Mb*transpose(Jc(1:12,1:6)); % multiplier of f in tau0
    Pinv_JcMinvSt_2feet   = pinvDamped(JcMinvSt_2feet,reg.pinvDamp); 
    JcDv_2feet            = JcDv(1:12);
   
    % nullJcMinvSt  = null space of PInv_JcMinvSt
    nullJcMinvSt_2feet    = eye(ROBOT_DOF) - Pinv_JcMinvSt_2feet*JcMinvSt_2feet;

    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    NLMbar_2feet          = nullJcMinvSt_2feet*Mbar;
    
    % Adaptation of control gains for back compatibility with older
    % versions of the controller
    impedances_2feet      = diag(impedances_old)*pinv(NLMbar_2feet,reg.pinvTol) + reg.impedances*eye(ROBOT_DOF);
    dampings_2feet        = diag(dampings_old)*pinv(NLMbar_2feet,reg.pinvTol)   + reg.dampings*eye(ROBOT_DOF); 
    
    % Terms used in Eq. 0)
    tauModel_2feet        = Pinv_JcMinvSt_2feet*(JcMinv_2feet*h -JcDv_2feet) +nullJcMinvSt_2feet*(h(7:end) -Mbj'/Mb*h(1:6) ...
                           -impedances_2feet*NLMbar_2feet*qTilde -ki_int_qtilde -dampings_2feet*NLMbar_2feet*qD);
    
    Sigma_2feet           = -(Pinv_JcMinvSt_2feet*JcMinvJct_2feet +nullJcMinvSt_2feet*JBar_2feet);
    
    % Desired rate-of-change of the robot momentum
    HDotDes              = [ m*xDDcomStar ;
                            -gain.DAngularMomentum*H(4:end)-gain.PAngularMomentum*intHw];

    % Contact wrenches realizing the desired rate-of-change of the robot
    % momentum HDotDes when standing on two feet. Note that f_HDot is
    % different from zero only when both foot are in contact, i.e. 
    % constraints(1) = constraints(2) = 1. This because when the robot
    % stands on one foot, the f_HDot is evaluated directly from the
    % optimizer (see next section). 
    pinvA_2feet     = pinv([AL,AR],reg.pinvTol);
         
    f_HDot_2feet    = pinvA_2feet*(HDotDes -gravityWrench)*constraints(1)*constraints(2);
                
    NA_2feet        = eye(12,12)-pinvA_2feet*([AL,AR]);
    SigmaNA_2feet   = Sigma_2feet*NA_2feet;
   
    % The optimization problem 1) seeks for the redundancy of the external
    % wrench that minimize joint torques. Recall that the contact wrench can 
    % be written as:
    %
    % f = f_HDot + NA*f_0 
    %
    % Then, the constraints on the contact wrench is of the form
    %
    % ConstraintsMatrix2Feet*f < bVectorConstraints,
    %
    % which in terms of f0 is:
    %
    % ConstraintsMatrix2Feet*NA*f0 < bVectorConstraints - ConstraintsMatrix2Feet*f_HDot
    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA_2feet;
    bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot_2feet;
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem 1).
    HessianMatrixQP2Feet      = SigmaNA_2feet'*SigmaNA_2feet + eye(size(SigmaNA_2feet,2))*reg.HessianQP;
    gradientQP2Feet           = SigmaNA_2feet'*(tauModel_2feet + Sigma_2feet*f_HDot_2feet);

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

    A1Foot                    =  AL*constraints(1)*(1-constraints(2)) + AR*constraints(2)*(1-constraints(1));
    HessianMatrixQP1Foot      =  A1Foot'*A1Foot + eye(size(A1Foot,2))*reg.HessianQP;
    gradientQP1Foot           = -A1Foot'*(HDotDes - gravityWrench);

    %% QP PARAMETERS FOR TWO FEET AND TWO LEGS BALANCING
    constraintMatrixLeftLeg   = ConstraintsMatrixLegs * blkdiag(w_R_l_leg',w_R_l_leg');
    constraintMatrixRightLeg  = ConstraintsMatrixLegs * blkdiag(w_R_r_leg',w_R_r_leg');
   
    ConstraintsMatrix2Feet2Legs    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot,constraintMatrixLeftLeg,constraintMatrixRightLeg);
    bVectorConstraints2Feet2Legs   = [bVectorConstraints;bVectorConstraints;bVectorConstraintsLegs;bVectorConstraintsLegs];
    
    % Terms used in Eq. 0)
    tauModel        = Pinv_JcMinvSt*(JcMinv*h -JcDv) +nullJcMinvSt*(h(7:end) -Mbj'/Mb*h(1:6) ...
                     -impedances*NLMbar*qTilde -ki_int_qtilde -dampings*NLMbar*qD);
    
    Sigma           = -(Pinv_JcMinvSt*JcMinvJct +nullJcMinvSt*JBar);

    % Contact wrenches realizing the desired rate-of-change of the robot
    % momentum HDotDes when standing on two feet. Note that f_HDot is
    % different from zero only when both foot are in contact, i.e. 
    % constraints(1) = constraints(2) = 1. This because when the robot
    % stands on one foot, the f_HDot is evaluated directly from the
    % optimizer (see next section). 
         
    f_HDot          = pinvA*(HDotDes -gravityWrench)*constraints(1)*constraints(2);

    SigmaNA         = Sigma*NA;
   
    % The optimization problem 1) seeks for the redundancy of the external
    % wrench that minimize joint torques. Recall that the contact wrench can 
    % be written as:
    %
    % f = f_HDot + NA*f_0 
    %
    % Then, the constraints on the contact wrench is of the form
    %
    % ConstraintsMatrix2Feet*f < bVectorConstraints,
    %
    % which in terms of f0 is:
    %
    % ConstraintsMatrix2Feet*NA*f0 < bVectorConstraints - ConstraintsMatrix2Feet*f_HDot
    ConstraintsMatrixQP2Feet2Legs  = ConstraintsMatrix2Feet2Legs*NA;
    bVectorConstraintsQp2Feet2Legs = bVectorConstraints2Feet2Legs-ConstraintsMatrix2Feet2Legs*f_HDot;
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem 1).
    HessianMatrixQP2Feet2Legs      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*reg.HessianQP;
    gradientQP2Feet2Legs           = SigmaNA'*(tauModel + Sigma*f_HDot);
    
    %% DEBUG DIAGNOSTICS
    % Unconstrained solution for the problem 1)
    %f0                       = -pinvDamped(SigmaNA,reg.pinvDamp*1e-5)*(tauModel + Sigma*f_HDot);
    % Unconstrained contact wrenches
    f                         = zeros(24,1); %pinvA*(HDotDes - gravityWrench) + NA*f0*constraints(1)*constraints(2); 
    % Error on the center of mass
    errorCoM                  = xcom - desired_x_dx_ddx_CoM(:,1);
    
end

