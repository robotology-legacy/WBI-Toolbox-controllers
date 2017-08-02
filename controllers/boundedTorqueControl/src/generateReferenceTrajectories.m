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

function refTrajectory  = generateReferenceTrajectories(qMin,qMax,t, model, ROBOT_DOF)

r = zeros(ROBOT_DOF,1);
rd = zeros(ROBOT_DOF,1);
rdd = zeros(ROBOT_DOF,1);

for i = 1 : ROBOT_DOF

    switch model.trajectory(i)
        case 1
            %Constant reference trajectory towards minimum/maximum joint limit
            if model.ramp(i) >= 0
                r(i) = qMax(i) - (qMax(i) - qMin(i)) * model.ratio(i);
            else
                r(i) = (qMax(i) - qMin(i)) * model.ratio(i) + qMin(i);
            end
            rd(i)    = 0;
            rdd(i)   = 0;

        case 2
            %Sinusoidal reference trajectory
            amplitude     =  ((qMax(i)-qMin(i))/2)/model.ratioAmplitude(i);
            r(i)          =    amplitude * sin( 2 * pi * model.rFrequency(i) * t + model.rPhase(i)) + (qMax(i)+qMin(i))/2;
            rd(i)         =    2 * pi * model.rFrequency(i) * amplitude * cos(2 * pi * model.rFrequency(i) * t + model.rPhase(i));
            rdd(i)        = - (2 * pi * model.rFrequency(i))^2 * amplitude * sin(2 * pi * model.rFrequency(i) * t + model.rPhase(i));

        case 3
            %Constant arbitrary reference position
            r(i)   = model.jointPosDes(i);
            rd(i)  = 0; 
            rdd(i) = 0;
            
        otherwise
            r(i) = 0; rd(i) = 0; rdd(i) = 0;
    end
    
end
    refTrajectory = [r, rd, rdd];
end