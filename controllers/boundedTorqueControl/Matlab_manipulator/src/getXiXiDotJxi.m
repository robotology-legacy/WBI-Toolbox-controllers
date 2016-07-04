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

function [xi,xiDot,J,JDot] = getXiXiDotJxi(jointPositions,jointVelocities,qmin, qmax)

    q0    = (qmin + qmax)/2;
    delta = (qmax - qmin)/2;

    xi = zeros(length(q0),1);
    for i = 1 : length(q0)
        if jointPositions(i) >= qmax(i)
            xi(i) = 1e2;
        elseif jointPositions(i) <= qmin(i)
            xi(i) = -1e2;
        else
            xi(i)    = atanh((jointPositions(i)-q0(i))./delta(i));
        end
    end
        
    J     = diag(delta./(cosh(xi).^2));
    
    xiDot = J\jointVelocities;
    
    JDot  = -2*diag(tanh(xi))*diag(jointVelocities);
end



%% Original function
% function [xi,xiDot,J,JDot] = getXiXiDotJxi(jointPositions,jointVelocities,qmin, qmax)
% 
%     q0    = (qmin + qmax)/2;
%     delta = (qmax - qmin)/2;
% 
%     xi    = atanh((jointPositions-q0)./delta);
%     
%     J     = diag(delta./(cosh(xi).^2));
%     
%     xiDot = J\jointVelocities;
%     
%     JDot  = -2*diag(tanh(xi))*diag(jointVelocities);
% end