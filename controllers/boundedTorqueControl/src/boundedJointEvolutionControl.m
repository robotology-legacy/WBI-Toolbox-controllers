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

function [r, e, tau, ksi_tilda] = boundedJointEvolutionControl(ROBOT_DOF, model, qmin, qmax, q, qd, M, h, g, C, refTrajectory, intKsi_tilda)
    %Computed torque control

    %Input:
    %ROBOT_DOF: number of degrees of freedom of the robot
    %model: parameter defined in the init script, containing control gains
    %qmin: vector of minimum joint limits
    %qmax: vector of maximum joint limits
    %q: vector of joint positions
    %qd: vector of joint velocities
    %M: inertia matrix
    %h: matrix combining gravity, coriolis and friction torques
    %refTrajectory: reference joint position trajectory r, in the shape [r; rd; rdd]
        %r: position reference trajectory
        %rd: velocity reference trajectory
        %rdd: acceleration reference trajectory
    %intKsi_tilda: integral of the position tracking error


    %Output:
    %r: position reference trajectory
    %e: position tracking error
    %tau: vector of input motor torques
    %ksi_tilda: position tracking error in terms of parameter ksi


    q0 = (qmin + qmax)/2;
    delta = (qmax - qmin)/2;

    r = refTrajectory(1 : ROBOT_DOF);
    rd = refTrajectory(ROBOT_DOF + 1 : 2 * ROBOT_DOF);
    rdd = refTrajectory(2 * ROBOT_DOF + 1 : 3 * ROBOT_DOF);
    e = q - r;
    ed = qd - rd;

    %convert reference trajectory and state in terms of ksi
    ksi = zeros(ROBOT_DOF, 1);
    ksid = zeros(ROBOT_DOF, 1);
    ksi_r = zeros(ROBOT_DOF, 1);
    ksi_rd = zeros(ROBOT_DOF, 1);
    ksi_rdd = zeros(ROBOT_DOF,1);

    for i = 1:ROBOT_DOF
        ksi_r(i) = atanh((r(i) - q0(i))/delta(i));
        ksi_rd(i) = rd(i) / (delta(i) * (1 - tanh(ksi_r(i))^2));
        ksi_rdd(i) = rdd(i)/(delta(i) * (1 - tanh(ksi_r(i))^2)) + 2 * tanh(ksi_r(i)) * ksi_rd(i)^2;

        ksi(i) = atanh((q(i) - q0(i))/delta(i));
        ksid(i) = qd(i) / (delta(i) * (1 - tanh(ksi(i))^2));
    end

    % get error on ksi
    ksi_tilda = ksi - ksi_r;
    ksi_tildad = ksid - ksi_rd;


    %% Control schemes: uncomment the one you wish to use

    % %Computed torque
    % ksiddstar = ksi_rdd - model.Kp * ksi_tilda - model.Kd * ksi_tildad - model.Ki * intKsi_tilda;
    % tau = M * ksiddstar + h; % h = g + C * qd;

    % %Gravity compensation, neglecting M*ksi_rdd and C*ksi_d
    J_ksi = diag(delta .* (1 - tanh(ksi).^2));
    tau = g - J_ksi\ (model.Kp * ksi_tilda + model.Kd * ksi_tildad + model.Ki * intKsi_tilda);

    % %Gravity compensation, including M*ksi_rdd and C*ksi_d
    % J_ksi = diag(delta .* (1 - tanh(ksi).^2));
    % J_ksid = diag( - 2 * delta .* tanh(ksi) .* (1 - tanh(ksi).^2) .* ksid);
    % tau = M * J_ksi * ksi_rdd + (M * J_ksid + C * J_ksi) * ksi_rd + g - J_ksi' \ model.Kp * ksi_tilda - J_ksi' \ model.Kd * ksi_tildad - J_ksi' \ model.Ki * intKsi_tilda;


    %Gravity compensation, no variable change to ksi
    % tau = M * rdd + C * rd + g - J_ksi' \ model.Kp * e - J_ksi' \ model.Kd * ed;
end