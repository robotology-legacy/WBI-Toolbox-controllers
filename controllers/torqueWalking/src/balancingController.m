function [HessianMatrixQP, biasVectorQP, ...
          feetAccelerationConstraintMatrix, feetAccelerationConstraintBoundVector, ...
          frictionConeConstraintMatrix, frictionConeUpperBoundVector, ...
          errorCoM] ...
         = balancingController( q0, qj, dqj, M, h, ... 
                                w_H_CoM_0, w_H_CoM, CoMVelocity, ...
                                w_H_l_sole, w_H_r_sole, w_J_l_sole, w_J_r_sole, ...
                                dJnu_l_sole, dJnu_r_sole, feetActivation, ...
                                frictionConeConstraintsMatrix, upperBoundFrictionConeConstraints, ...
                                ROBOT_DOF, gain)

 %% BALANCING CONTROLLER
    %
    % ---------------------------------------------------------------------
    % Modelling
    % ---------------------------------------------------------------------
    % Let
    %
    % 1)  u := [jointTorques,
    %           contactForces] \in R^{nDof+6nc}
    %
    % Then, the equations of motion in the centroidal coordinates write
    %
    % 2)  M(q)dnu + h(q,nu) = B * u
    %
    % with M block diagonal and
    %
    % 3a) B   := [S, Jc^t], 
    %
    % 3b) S   := [0_{6xnDof}
    %             eye(nDof)],
    %
    % 3c) Jc  := [jacobianLeftFoot*deltaL
    %             jacobianRightFoot*deltaR],
    %
    % with deltaL and deltaR two "activating" functions being either 0 or 1
    % depending on contact conditions (such as measured forces).
    % The holonomic constraints acting on the system represent the fact
    % that the robot's feet do not move. They can be compactly written as
    % 
    % 4) dJc * nu + Jc * dnu = 0,
    %
    % i.e. the feet accelerations are equal to zero.
    %
    % ---------------------------------------------------------------------
    % Control objective
    % ---------------------------------------------------------------------
    % The control objective is the stabilization of nu.
    %
    % Hence, we would like to find u such that B * u - h = feedback action
    % on joint control while satisfying the equality constraint 4).
    %
    % In the language of optimization theory, we solve
    %
    % 5) u* = argmin (1/2) * |inv(M) * (B * u - h) - feedback|^2
    %           s.t.
    %               C * u < b
    %               dJc * nu + Jc * inv(M) * (B * u - h) = 0
    %               feedback = ddnu_desired - kp * nuTilde - kd * dnuTilde
    %
    % note that the objective function could be rewritten as 1/2*|dnu - dnu_desired|^2
    % 
    % Re-formulated as a QP:
    %
    % 6) u* = argmin 1/2 * u' (inv(M) * B)' * (inv(M) * B) * u - u' * (inv(M) * B)' * (-inv(M) * h - feedback)
    %           s.t.
    %               C * u < b
    %               (Jc * inv(M) * B) * u = Jc * inv(M) * h - dJc * nu
    %               feedback = ddnu_desired - kp * nuTilde - kd * dnuTilde

%Contact Jacobians
Jc       = [w_J_l_sole * feetActivation(1);
            w_J_r_sole * feetActivation(2)];

dJc_nu   = [dJnu_l_sole * feetActivation(1);
            dJnu_r_sole * feetActivation(2)];
        
%Selection matrices
S        = [zeros(6,ROBOT_DOF);
            eye(ROBOT_DOF) ];
B        = [S, Jc'];
Storques = [eye(ROBOT_DOF)   zeros(ROBOT_DOF,12)];
% Sforces  = [zeros(12, ROBOT_DOF) eye(12)];


%% Keep joint positions close to their initial position
desired_q_dq_ddq = [q0, zeros(ROBOT_DOF,2)];
dqq_star = desired_q_dq_ddq(:,3) - gain.joints.d * (dqj - desired_q_dq_ddq(:,2)) - gain.joints.p * (qj - desired_q_dq_ddq(:,1));

%% Desired CoM linear acceleration
desired_x_dx_ddx_CoM = [w_H_CoM_0(1:3, 4), zeros(3,2)];
xCoM             = w_H_CoM(1:3, 4);
dxCoM            = CoMVelocity(1:3);
ddot_x_star      = desired_x_dx_ddx_CoM(:,3) - gain.x_CoM.d * (dxCoM - desired_x_dx_ddx_CoM(:,2)) - gain.x_CoM.p * (xCoM - desired_x_dx_ddx_CoM(:,1)); 

%% Desired CoM angular acceleration
desired_Intw_w_dw_CoM = [w_H_CoM(1:3,1:3), zeros(3,2)];
w_R_CoM = w_H_CoM(1:3,1:3);
w_CoM = CoMVelocity(4:6);
dw_star = rotationalPID(w_R_CoM,w_CoM,desired_Intw_w_dw_CoM, [gain.w_CoM.p, gain.w_CoM.d]); 

%% Desired action feedback
dot_nu_star     = [ddot_x_star; dw_star; dqq_star];

%convert to QP objective function
HessianMatrixQP = (M \ B)' * (M \ B) +  gain.reg.joint_torques * (Storques' * Storques);
biasVectorQP    = (M \ B)' * (- M \ h - dot_nu_star);

%Acceleration constraint for foot(feet) in contact with the ground
feetAccelerationConstraintMatrix      = Jc / M * B;
feetAccelerationConstraintBoundVector = Jc / M * h - dJc_nu;

%Friction cone constraints for foot(feet) in contact with the ground
frictionConeConstraintMatrixLeftFoot  = frictionConeConstraintsMatrix * blkdiag(w_H_l_sole(1:3,1:3)', w_H_l_sole(1:3,1:3)');
frictionConeConstraintMatrixRightFoot = frictionConeConstraintsMatrix * blkdiag(w_H_r_sole(1:3,1:3)', w_H_r_sole(1:3,1:3)'); 

frictionConeConstraintMatrix = [zeros(length(upperBoundFrictionConeConstraints), ROBOT_DOF), frictionConeConstraintMatrixLeftFoot, zeros(length(upperBoundFrictionConeConstraints),6);
                                zeros(length(upperBoundFrictionConeConstraints), ROBOT_DOF), zeros(length(upperBoundFrictionConeConstraints),6), frictionConeConstraintMatrixRightFoot];

frictionConeUpperBoundVector = [upperBoundFrictionConeConstraints * feetActivation(1);
                                upperBoundFrictionConeConstraints * feetActivation(2)];

%Debug information
errorCoM = xCoM - desired_x_dx_ddx_CoM(:,1);

