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

%t: simulation time
%r, rd, rdd: reference trajectory
%knee joint limits -2.1817:0.4014

function refTrajectory  = generateReferenceTrajectories(t, model)
r = zeros(model.n,1);
rd = r;
rdd = r;

switch model.trajectory
    case 1
        for i = 1 : model.n
            %Ramped reference trajectory towards minimum/maximum joint limit
            r(i) = model.p(i) * t + model.r0(i);
            rd(i) = model.p(i);
            rdd(i) = 0;
            if r(i) >= model.rmax(i)
                r(i) = model.rmax(i);
                rd(i) = 0;
            end
            if r(i) <= model.rmin(i)
                r(i) = model.rmin(i);
                rd(i) = 0;
            end
        end
        
    case 2
        %2-DOF manipulator case:
        %Joint 1 has constant reference trajectory
        %Joint 2 has sinusoidal reference trajectory       
        for i = 1 : model.n
            r(i) = model.rAmplitude(i) * sin(model.rFrequency(i) * t) + model.rbias(i);
            rd(i) = model.rFrequency(i) * model.rAmplitude(i) * cos(model.rFrequency(i) * t);
            rdd(i) = - model.rFrequency(i)^2 * model.rAmplitude(i) * sin(model.rFrequency(i) * t);
        end
    
    otherwise
        for i = 1 : model.n
            r(i) = 0; rd(i) = 0; rdd(i) = 0;
        end
end

refTrajectory = [r, rd, rdd];
end