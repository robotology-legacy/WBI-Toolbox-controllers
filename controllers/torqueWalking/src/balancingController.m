function [HessianMatrixQP, biasVectorQP, ...
          feetAccelerationConstraintMatrix, feetAccelerationConstraintBoundVector, ...
          frictionConeConstraintMatrix, frictionConeUpperBoundVector] ...
         = balancingController( M, h, dnu_star, ...
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

%QP objective function
HessianMatrixQP = (M \ B)' * (M \ B) +  gain.reg.joint_torques * (Storques' * Storques);
biasVectorQP    = (M \ B)' * (- M \ h - dnu_star);

%Acceleration constraint for foot(feet) in contact with the ground
feetAccelerationConstraintMatrix      = Jc / M * B;
feetAccelerationConstraintBoundVector = Jc / M * h - dJc_nu;

%Friction cone constraints for foot(feet) in contact with the ground
    % Update constraint matrices. The constraint matrix for the inequality
    % constraints in the problem 1) is built up starting from the constraint
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
    % constraintMatrixLeftFoot = ConstraintsMatrix*l_sole_R_w
    %
    % The same holds for the right foot
    
frictionConeConstraintMatrixLeftFoot  = frictionConeConstraintsMatrix * blkdiag(w_H_l_sole(1:3,1:3)', w_H_l_sole(1:3,1:3)');
frictionConeConstraintMatrixRightFoot = frictionConeConstraintsMatrix * blkdiag(w_H_r_sole(1:3,1:3)', w_H_r_sole(1:3,1:3)'); 

frictionConeConstraintMatrix = [zeros(length(upperBoundFrictionConeConstraints), ROBOT_DOF), frictionConeConstraintMatrixLeftFoot, zeros(length(upperBoundFrictionConeConstraints),6);
                                zeros(length(upperBoundFrictionConeConstraints), ROBOT_DOF), zeros(length(upperBoundFrictionConeConstraints),6), frictionConeConstraintMatrixRightFoot];

frictionConeUpperBoundVector = [upperBoundFrictionConeConstraints * feetActivation(1);
                                upperBoundFrictionConeConstraints * feetActivation(2)];

