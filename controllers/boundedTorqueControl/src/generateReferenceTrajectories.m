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

function refTrajectory  = generateReferenceTrajectories(qMin,qMax,t, model)
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
            amplitude  = ((qMax-qMin)/2)/model.ratioAmplitude;
            r          =    amplitude * sin( 2 * pi * model.rFrequency * t) + (qMax+qMin)/2;
            rd         =    2 * pi * model.rFrequency * amplitude * cos(2 * pi * model.rFrequency * t);
            rdd        = - (2 * pi * model.rFrequency)^2 * amplitude * sin(2 * pi * model.rFrequency * t);

        otherwise
            r = 0; rd = 0; rdd = 0;
    end

    refTrajectory = ones(length(qMin),1)*[r, rd, rdd];
end