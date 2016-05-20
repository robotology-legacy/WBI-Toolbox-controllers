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

function J_xid = getJ_xid(ROBOT_DOF, qmin, qmax, refTrajectory)

    %Input:
    %ROBOT_DOF: number of degrees of freedom of the robot
    %model: parameter defined in the init script, containing control gains
    %qmin: vector of minimum joint limits
    %qmax: vector of maximum joint limits
    %q: vector of joint positions
    %qd: vector of joint velocities
    %refTrajectory: reference joint position trajectory r, in the shape [r; rd; rdd]
        %r: position reference trajectory
        %rd: velocity reference trajectory
        %rdd: acceleration reference trajectory

    %Output:
    %r: position reference trajectory
    %e: position tracking error
    %xi_tilda: position tracking error in terms of parameter xi

    q0 = (qmin + qmax)/2;
    delta = (qmax - qmin)/2;

    r = refTrajectory(1 : ROBOT_DOF);
    rd = refTrajectory(ROBOT_DOF + 1 : 2 * ROBOT_DOF);

    %convert reference trajectory and state in terms of xi
    xi_r = zeros(ROBOT_DOF, 1);
    xi_rd = zeros(ROBOT_DOF, 1);

    for i = 1:ROBOT_DOF     
        xi_r(i) = atanh((r(i) - q0(i))/delta(i));
        xi_rd(i) = rd(i) / (delta(i) * (1 - tanh(xi_r(i))^2));
    end
    
    J_xid = delta * (1 - tanh(xi_r)^2) * xi_rd;
end