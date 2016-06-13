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
    switch model.trajectory
        case 1
            %Ramped reference trajectory towards minimum/maximum joint limit
            r = model.p * t + model.r0;
            rd = model.p;
            rdd = 0;
            if r >= model.rmax
                r = model.rmax;
                rd = 0;
            end
            if r <= model.rmin
                r = model.rmin;
                rd = 0;
            end

        case 2
            %Sinusoidal reference trajectory
            r = model.rAmplitude * sin(model.rFrequency * t) + model.rbias;
            rd = model.rFrequency * model.rAmplitude * cos(model.rFrequency * t);
            rdd = - model.rFrequency^2 * model.rAmplitude * sin(model.rFrequency * t);

        otherwise
            r = 0; rd = 0; rdd = 0;
    end

    refTrajectory = [r, rd, rdd];
end