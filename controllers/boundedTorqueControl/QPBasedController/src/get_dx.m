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

function [x, dx, ddx] = get_dx(J, dJNu, H, Nu, dNu)

% %Compute end effector position, velocity and acceleration in world coordinates

% %Inputs:
% Nu: base and joint velocity [base velocity; joint velocities]
% dNu: base and joint acceleration [base acceleration; joint accelerations]
%      optional - omit if don't need to compute ddx
% H: transformation matrix of end effector position in world coordinates
% J: robot Jacobian
% dJNu: time derivative of the Jacobian multiplied by Nu
%       optional - omit if don't need to compute ddx

% %Outputs:
% x: end effector position in world coordinates
% xd: end effector velocity
% xdd: end effector acceleration
% x has the following shape: 
%     x = [x; y; z; theta_x, theta_y, theta_z];


    %convert transformation matrix into world coordinates
    x = zeros(6,1);
    x(1:3) = H(1:3, 4);    
    x(4) = atan2(H(3,2), H(3,3)); % 0;
    x(5) = atan2(-H(3,1), sqrt(H(3,2)^2 + H(3,3)^2)); %x(5) = acos(H(1,1));
    x(6) = atan2(H(2,1), H(1,1)); % 0;

    dx = J * Nu;

    if ~isempty(dNu)
        ddx = J * dNu + dJNu;
    else
        ddx = zeros(size(dx));
    end

end