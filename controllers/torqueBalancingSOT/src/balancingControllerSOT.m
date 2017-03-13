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
                                                               jointAngles,desJointAngles, massMatrix, biasTorques, ...
                                                               jacobians, jacobiansDotNu, robotVelocity,...
                                                               poseLeftFoot, poseRightFoot, desiredTaskAcc, taskAccError, impedances,dampings,gain,CONFIG, reg)
          
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
    %
    % and aTaskDes such that the output functions i-iv are stabilized
    % towards desired values.
    % 
    % Now, the variable u0 is found such that the joint dynamics is imposed
    % at desired values. 
    
    % The mass matrix M can be decomposed into blocks in the shape
    % M = [Mb   Mbj]
    %     [MbJ' Mj ]
    % such that equation 2) takes the form
    % 8) [Mb   Mbj] * [xDD ]  + [h_B] = [(Bu)_B]
    %    [Mbj' Mj ]   [qjDD]    [h_j]   [(Bu)_j]
    % from which a new equation of the same form as 2) is obtained :
    % 9) MjBar*qjDD + hjBar = Bbar*u 
    % with 
    % MjBar = Mj - Mbj'*inv(Mb)*Mbj
    % hjBar = hj - Mbj'*inv(Mb)*h_B
    %  Bbar = [- Mbj'*inv(Mb)]
    %         [  eye(n)      ]
    % such that qjDD = inv(MjBar) * (Bbar*u - hjBar)
    %
    % Hence, we would like to find u such that Bbar * u - hjBar = feedback action
    % on joint control while satisfying the equality constraint 7).
    %
    % In the language of optimization theory, we solve
    %
    % 11) u* = argmin (1/2)*|S^t * inv(M) * (Bu - h) + tauFeedback|^2     %To do computed torque --|Bbar * u - hjBar + tauFeedback|^2
    %           s.t.
    %               Cu < b
    %               JDot * nu + J * inv(M) * (Bu - h) = aTaskDes
    %               tauFeedback = kp * qTilde + kd * qTildeDot
    %
    % and aTaskDes such that the output functions i-iv are stabilized
    % towards desired values.
    %% IMPLEMENTATION
    
    ROBOT_DOF                 = size(ROBOT_DOF_FOR_SIMULINK,1);
    S                         = [zeros(6,ROBOT_DOF);
                                 eye(ROBOT_DOF)   ];
    
    contactJacobians          = [jacobians(end-11:end-6,:)*constraints(1);
                                 jacobians(end- 5:end,  :)*constraints(2)];
                   
    B                         = [S,contactJacobians'];
    tauFeedback               = diag(impedances)*(jointAngles-desJointAngles)...
                                + diag(dampings)*robotVelocity(7:end);

    if CONFIG.QP.USE_STRICT_TASK_PRIORITIES
        hessianMatrix         = ((S' / massMatrix)* B)' * ((S' / massMatrix)* B);
        biasVector            = ((S' / massMatrix)* B)' * tauFeedback - ((S' / massMatrix) * B)' * (S' / massMatrix) * biasTorques;
    else
        % In this case, the optimization problem 11) is changed, and the equality
        % constraint 
        %
        % 12a) aStar + J * inv(M) * Bu  = 0 
        % 12b) aStar = JDot * nu  - aTaskDes - J * inv(M) * h
        %
        % is put in the const function, i.e.
        %
        % 11) u* = argmin (1/2)*wP*|S^t * inv(M) * (Bu - h) + tauFeedback|^2 + (1/2)*wT*|aStar + J * inv(M) * Bu|^2
        %           s.t.
        %               Cu < b
        %               tauFeedback = kp * qTilde + kd * qTildeDot
        %
        
        
        aStar = jacobiansDotNu  - desiredTaskAcc - (jacobians / massMatrix) * biasTorques;
        
        hessianMatrix             = gain.weightPostural*((S' / massMatrix)* B)' * ((S' / massMatrix)* B) ...
                                  + gain.weightTasks*((jacobians / massMatrix) * B)' * (jacobians / massMatrix) * B;
                              
        biasVector                = gain.weightPostural*(((S' / massMatrix)* B)' * tauFeedback - ((S' / massMatrix) * B)' * (S' / massMatrix) * biasTorques)...
                                  + gain.weightTasks*((jacobians / massMatrix) * B)'*aStar;
        
    end
    
    biasVector = biasVector ...
               + eye(size((S' * B)')) * reg.jointAnglesQP * (jointAngles-desJointAngles) ...
               + eye(size((S' * B)')) * reg.torquesQP * tauFeedback ...
               + eye(size(((jacobians / massMatrix) * B)')) * reg.taskAccQP * (taskAccError);
           
    constraintMatrixEq        = jacobians*(massMatrix\B);
    upperBoundEqConstraints   = desiredTaskAcc - jacobiansDotNu +  (jacobians/massMatrix)*biasTorques; 
    
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
    
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(poseLeftFoot(1:3,1:3)' ,poseLeftFoot(1:3,1:3)');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(poseRightFoot(1:3,1:3)',poseRightFoot(1:3,1:3)');
    
    

end

