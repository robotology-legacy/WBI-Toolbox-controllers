%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Marie Charbonneau, Daniele Pucci
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

function [tau, e_ref] = jointEvolutionControl(q_ref, q, dq, M, h, ddq_r, dq_r, q_r, int_e, ROBOT_DOF, model)

% % Compute motor torque with gravity compensation control

% %Inputs:
% q_ref: reference joint trajectory (the initial trajectory defined before conversion to end effector position) 
% q: joint position
% dq: joint velocity
% M: Mass matrix
% h: bias forces
% ddq_r: desired joint acceleration (obtained from inverse kinematics)
% dq_r: desired joint velocity
% q_r: desired joint position
% int_e: integral of error between joint position and desired position
% ROBOT_DOF: parameter, number of degrees of freedom of the robot
% model: parameter, contains the controller gains Kp, Kd, Ki

% %Outputs:
% tau: motor torque
% e_ref: tracking error from reference trajectory q_ref


%Keep components related to joints
hj = h(7 : 6 + ROBOT_DOF); %h = C*dq_r + g
Mj = M(7 : 6 + ROBOT_DOF, 7 : 6 + ROBOT_DOF);

%Joint tracking error
e = q - q_r;
de = dq - dq_r;

%Torque
%Gravity compensation torque including M and C terms (h = Cq_r + g)
tau = Mj * ddq_r + hj - model.Kp * e - model.Kd * de - model.Ki * int_e;

%Error between reference joint trajectory and actual trajectory
e_ref = q - q_ref;

end