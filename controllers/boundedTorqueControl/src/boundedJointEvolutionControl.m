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

function [xi, xiTilde, xiDot, jointReference, jointError, tau] =  boundedJointEvolutionControl(model, qmin, qmax, jointPositions, jointVelocities, M, g, CJXiDotDes,CrDot,  refTrajectory, intKsi_tilda)
    %Computed torque control

    [jointReference,jointReferenceDot,jointReferenceDDot]       = getReferences(refTrajectory);
    
    [xi,xiDot,J,JDot]    = getXiXiDotJxi(jointPositions,jointVelocities,qmin, qmax);
    
    [xiDes,xiDotDes,JDes,JDotDes] ...
                         = getXiXiDotJxi(jointReference,jointReferenceDot,qmin, qmax);

    xiDDotDes            = JDes\(jointReferenceDDot - JDotDes*xiDotDes);
    
    xiTilde              = xi    - xiDes;
    
    xiTildeDot           = xiDot - xiDotDes;

    tau                  = g + M*J*xiDDotDes + (M*JDot*xiDotDes + CJXiDotDes) - model.Kp.*xiTilde - model.Kd.*xiTildeDot; 
    
    jointError           = jointPositions  - jointReference;

    if ~model.variableChange
        jointErrorDot    = jointVelocities - jointReferenceDot; 
        tau              = g + M*jointReferenceDDot + CrDot - model.Kp.*jointError - model.Kd.*jointErrorDot; 
    end
        
end