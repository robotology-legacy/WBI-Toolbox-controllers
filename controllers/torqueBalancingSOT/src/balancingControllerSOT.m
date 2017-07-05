%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @authors: Daniele Pucci & Marie Charbonneau
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

function [hessianMatrix,biasVector,constraintMatrixLeftFoot,constraintMatrixRightFoot,constraintMatrixEq, upperBoundEqConstraints] = ...
                                        balancingControllerSOT(constraints, impedances, dampings, jointAngles, ...
                                                               massMatrix, biasTorques, jacobiansDotNu, poseLeftFoot, poseRightFoot, jacobians, robotVelocity, ...
                                                               desJointAngles, desiredTaskAcc, ...
                                                               ROBOT_DOF_FOR_SIMULINK, ConstraintsMatrix, ...
                                                               gain, CONFIG)
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
    % 2)  M(q)nuD + h(q,nu) = Bu
    %
    % with M block diagonal and
    %
    % 3a) B   := [S,J_c^t], 
    %
    % 3b) S   := [0_{6xnDof}
    %             eye(nDof)],
    %
    % 3c) J_c := [jacobianLeftFoot*deltaL
    %             jacobianRightFoot*deltaR],
    %
    % with deltaL and deltaR two "activating" functions being either 0 or 1
    % depending on contact conditions such as measured forces, etc.
    % The holonomic constraints acting on the system represent the fact
    % that the robot's feet do not move. They can be compactly written as
    % 
    % 4) J_cDot*nu + J_c * nuDot = 0,
    %
    % i.e. the feet accelerations are equal to zero.
    %
    % ---------------------------------------------------------------------
    % Control objective
    % ---------------------------------------------------------------------
    % The control objective is the stabilization of the following outputs:
    %
    %   i) Center of mass;
    %  ii) Orientation of the base;
    % iii) Pose of left foot;
    %  iv) Pose of right foot.
    %
    % Now, let J_G, J_B, J_R, J_L, denote the jacobians of the center of
    % mass, base orientation, right, and left foot frame, respectively. 
    % Define
    %
    %
    %  5) J := [J_G;
    %           J_B;
    %           J_L;
    %           J_R];
    %
    % Therefore, the acceleration of the outputs i-iv stuck together are:
    %
    %  6) JDot * nu + J * nuDot = aOut
    %
    % In view of Eq. 2), one has:
    %
    %  7) JDot * nu + J * inv(M) * (Bu - h) = aTask
   
    % and aTaskDes such that the output functions i-iv are stabilized
    % towards desired values.
    % 
    % Now, the variable u0 is found such that the joint dynamics is imposed
    % at desired values. 
    %
    % Hence, we would like to find u such that B * u - h = feedback action
    % on joint control while satisfying the equality constraint 7).
    %
    % In the language of optimization theory, we solve
    %
    % 11) u* = argmin (1/2)*|S^t * inv(M) * (B * u - h) + tauFeedback|^2
    %           s.t.
    %               Cu < b
    %               JDot * nu + J * inv(M) * (B * u - h) = aTaskDes
    %               tauFeedback = kp * qTilde + kd * qTildeDot
    %
    % and aTaskDes such that the output functions i-iv are stabilized
    % towards desired values.
    %% IMPLEMENTATION
    
    ROBOT_DOF                 = size(ROBOT_DOF_FOR_SIMULINK,1);
    S                         = [zeros(6,ROBOT_DOF);
                                 eye(ROBOT_DOF)   ];

    Storques                  = [zeros(ROBOT_DOF,12) eye(ROBOT_DOF)];
    Sforces                   = [eye(12)             zeros(12, ROBOT_DOF)];
    
    contactJacobians          = [jacobians(end-11:end-6,:)*constraints(1);
                                 jacobians(end- 5:end,  :)*constraints(2)];
                   
    B                         = [S,contactJacobians'];
    tauFeedback               = diag(impedances)*(jointAngles-desJointAngles)...
                                + diag(dampings)*robotVelocity(7:end);
                            
    St_invM_B                 = (S' / massMatrix)* B;
    jacobians_invM            = jacobians / massMatrix; 
    jacobians_invM_biasTorques= jacobians_invM * biasTorques;
    jacobians_invM_B          = jacobians_invM * B;                            

    if CONFIG.QP.USE_STRICT_TASK_PRIORITIES
        hessianMatrix             = St_invM_B' * St_invM_B;
        biasVector                = St_invM_B' * tauFeedback - St_invM_B' * (S' / massMatrix) * biasTorques;
        constraintMatrixEq        = jacobians_invM_B; 
        upperBoundEqConstraints   = desiredTaskAcc - jacobiansDotNu + jacobians_invM_biasTorques;
    else
        % In this case, the optimization problem 11) is changed, and the equality
        % constraint 
        %
        % 12a) aStar + J * inv(M) * B * u  = 0 
        % 12b) aStar = JDot * nu  - aTaskDes - J * inv(M) * h
        %
        % is put in the const function, i.e.
        %
        % 11) u* = argmin (1/2)*wP*|S^t * inv(M) * (B * u - h) + tauFeedback|^2 + (1/2)*wT*|aStar + J * inv(M) * B * u|^2
        %           s.t.
        %               C * u < b
        %               tauFeedback = kp * qTilde + kd * qTildeDot
        %
        
        aStar                     = jacobiansDotNu  - desiredTaskAcc - jacobians_invM_biasTorques;
        
        hessianMatrix             = gain.weightPostural   * (St_invM_B' * St_invM_B) ...
                                  + gain.weightTasks      * (jacobians_invM_B' * jacobians_invM_B) ...
                                  + gain.weightMinTorques * (Storques' * Storques) ...
                                  + gain.weightMinContactForces * (Sforces' * Sforces);
                              
        biasVector                = gain.weightPostural * (St_invM_B' * tauFeedback - St_invM_B' * (S' / massMatrix) * biasTorques)...
                                  + gain.weightTasks    * (jacobians_invM_B' * aStar);   
                          
        constraintMatrixEq        = jacobians_invM_B; 
        upperBoundEqConstraints   = -aStar; %desiredTaskAcc - jacobiansDotNu + jacobians_invM_biasTorques;
    end   
    
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
    
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(poseLeftFoot(1:3,1:3)' , poseLeftFoot(1:3,1:3)');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(poseRightFoot(1:3,1:3)', poseRightFoot(1:3,1:3)');
    
    

end

