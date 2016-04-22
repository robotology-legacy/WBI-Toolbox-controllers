
referenceParams.noOscillationTime        =  0;        % time that the robot waits before starting the left-and-righ
referenceParams.directionOfOscillation   = [0;0;0];
referenceParams.amplitudeOfOscillation   =  0.0;  
referenceParams.frequencyOfOscillation   =  0.0;

if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2 )
    
    if  CONFIG.DEMO_MOVEMENTS==1
        referenceParams.directionOfOscillation  = [0;1;0];
        referenceParams.amplitudeOfOscillation  =  0.035;
        referenceParams.frequencyOfOscillation  =  0.35;

    end
else
    if  CONFIG.DEMO_MOVEMENTS==1
        referenceParams.directionOfOscillation  = [0;1;0];
        referenceParams.amplitudeOfOscillation  =  0.015;
        referenceParams.frequencyOfOscillation  =  0.25;
    end
end
