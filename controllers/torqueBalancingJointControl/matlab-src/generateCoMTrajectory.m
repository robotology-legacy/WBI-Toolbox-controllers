function  desired_x_dx_ddx_CoM   = generateCoMTrajectory(xCoM_ini, t, referenceParams)
%% Time before the oscillation starts
if t >= referenceParams.noOscillationTime
    
    Amp = referenceParams.amplitudeOfOscillation;
    
else
    
    Amp = 0;

end

%% Trajectory generator
 freq       = referenceParams.frequencyOfOscillation;

 xCoMDes    =  xCoM_ini + Amp*sin(2*pi*freq*t)*referenceParams.directionOfOscillation;
 
 dxCoMDes   =  Amp*2*pi*freq*cos(2*pi*freq*t)*referenceParams.directionOfOscillation;
 
 ddxCoMDes  = -Amp*(2*pi*freq)^2*sin(2*pi*freq*t)*referenceParams.directionOfOscillation;


 desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
 
end
