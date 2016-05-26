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

function [ddq, dq, q] = qpInverseKinematics(qmin, qmax, q, M, h, J, x, dx, x_r, dx_r, ddx_r, q_t, Ts, ROBOT_DOF, model)

% %Compute the desired joint trajectory, from desired end effector position,
% %using QP solver

% %Inputs:
% qmin: lower joint limit (position)
% qmax: upper joint limit (position)
% q: joint position
% M: mass matrix
% h: bias forces
% J: robot jacobian
% x: end effector position (world coordinates)
% dx: end effector velocity (world coordinates)
% x_r: reference end effector psoition (world coordinates)
% dx_r: reference end effector velocity (world coordinates)
% ddx_r: reference end effector acceleration (world coordinates)
% q_t: joint position at previous time instant
% Ts: sample time
% ROBOT_DOF: parameter, number of degrees of freedom of the robot
% model: parameter, contains the gains Kpx, Kdx, mu_p, eta_p

% %Outputs:
% ddq: joint acceleration
% dq: joint velocity
% q: joint position


%Tracking error
e = x - x_r;
de = dx - dx_r;

%Target end effector acceleration in world coordinates (do t dot r)
ddr = ddx_r - model.Kpx * e - model.Kdx * de;
dr = dx_r - model.Kpx * e;

%QP solver to get dot Nu (target base and joint acceleration)
dnu = zeros(6 + ROBOT_DOF, 1);
coder.extrinsic('qpOASES')
%Joint limit upper and lower bounds for acceleration
ksi_min = [-1E3 * ones(6,1); model.mu_p * (model.eta_p * qmin - q)];
ksi_max = [ 1E3 * ones(6,1); model.mu_p * (model.eta_p * qmax - q)];
dnu = qpOASES(M^2, (M*h), J, ksi_min, ksi_max, [], ddr);


%QP solver to get Nu (target base and joint velocity)
nu = zeros(6 + ROBOT_DOF, 1);
%Joint limit upper and lower bounds for velocity
ksi_min = [-1E3 * ones(6,1); model.mu_p * (qmin - q)];
ksi_max = [ 1E3 * ones(6,1); model.mu_p * (qmax - q)];
nu = qpOASES(M, zeros(6 + ROBOT_DOF, 1), J, ksi_min, ksi_max, [], dr);

%Keep only joint acceleration and velocity (leave out the base)
ddq = dnu(7 : 6 + ROBOT_DOF);
dq = nu(7 : 6 + ROBOT_DOF);

%Joint position
q = q_t + dq * Ts + ddq * Ts^2/2;

end
