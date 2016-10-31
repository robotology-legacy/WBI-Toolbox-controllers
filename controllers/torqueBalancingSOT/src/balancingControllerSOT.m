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

function [hessianMatrix,biasVector,constraintMatrixLeftFoot,constraintMatrixRightFoot,constraintMatrixEq, upperBoundEqConstraints] = ...
                                        balancingControllerSOT(constraints,ROBOT_DOF_FOR_SIMULINK,ConstraintsMatrix,...
                                                               jointAngles,desJointAngles, massMatrix, biasTorques, gravityTorques, jacobians, jacobiansDotNu, robotVelocity, poseLeftFoot, poseRightFoot, desiredTaskAcc,impedances)
          
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
    %  iv) POse of right foot.
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
    %
    % and aTaskDes such that the output functions i-iv are stabilized
    % towards desired values.
    %
    % Now, the variable u0 is found such that the joint dynamics is imposed
    % at desired values. Being the mass matrix block diagonal, from 2) we
    % have
    %
    % 8) Mj(q)qjDD + S^t * h(q,nu) = S^t * Bu
    %
    % Hence, we would like to find u such that S^t * Bu = feedback action
    % on joint control while satisfying the equality constraint 7).
    %
    % In the language of optimization theory, we solve
    %
    % 11) u* = argmin (1/2)*|S^t * Bu + tauFeedback|^2
    %           s.t.
    %               Cu < b
    %               JDot * nu + J * inv(M) * (Bu - h) = aTaskDes
    %               tauFeedback = kp * qTilde + kd * qTildeDot
    % and aTaskDes such that the output functions i-iv are stabilized
    % towards desired values.
    %% IMPLEMENTATION
    
    
    ROBOT_DOF                 = size(ROBOT_DOF_FOR_SIMULINK,1);
    S                         = [zeros(6,ROBOT_DOF);
                                 eye(ROBOT_DOF,ROBOT_DOF)];
    
    contactJacobians          = [jacobians(end-11:end-6,:)*constraints(1);
                                 jacobians(end- 5:end,  :)*constraints(2)];
                   
    B                         = [S,contactJacobians']; 
    tauFeedback               = diag(impedances)*(jointAngles-desJointAngles);% + diag(damping)*robotVelocity(7:end);

    hessianMatrix             = (S' * B)' * S' * B ;
    biasVector                = (S' * B)' * tauFeedback;
    
    constraintMatrixEq        = jacobians*(massMatrix\B);
    upperBoundEqConstraints   = desiredTaskAcc - jacobiansDotNu +  (jacobians/massMatrix)*biasTorques; 
    
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
    % constraintMatrixLeftFoot = ConstraintsMatrix*l_sole_R_w
    %
    % The same hold for the right foot
    
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(poseLeftFoot(1:3,1:3)' ,poseLeftFoot(1:3,1:3)');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(poseRightFoot(1:3,1:3)',poseRightFoot(1:3,1:3)');
    
    

end

