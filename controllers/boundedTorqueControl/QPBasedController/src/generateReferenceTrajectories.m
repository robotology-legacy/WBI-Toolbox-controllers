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

function [q_r, dq_r, ddq_r]  = generateReferenceTrajectories(t, model)

%t: simulation time
%r, rd, rdd: reference trajectory
%knee joint limits -2.1817:0.4014

switch model.trajectory
  
    case 1
        %Ramped reference trajectory towards minimum/maximum joint limit
        q_r = model.p * t + model.r0;
        dq_r = model.p;
        ddq_r = 0;
        if q_r >= model.rmax
            q_r = model.rmax;
            dq_r = 0;
        end
        if q_r <= model.rmin
            q_r = model.rmin;
            dq_r = 0;
        end
        
    case 2
        %Sinusoidal reference trajectory
        q_r = model.rAmplitude * sin(model.rFrequency * t) + model.rbias;
        dq_r = model.rFrequency * model.rAmplitude * cos(model.rFrequency * t);
        ddq_r = - model.rFrequency^2 * model.rAmplitude * sin(model.rFrequency * t);
        
    otherwise
        q_r = 0; dq_r = 0; ddq_r = 0;
end