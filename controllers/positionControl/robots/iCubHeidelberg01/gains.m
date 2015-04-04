ROBOT_DOF = 15;

amplTorso            = [ 15  -15   10 ]; 
amplLeftLeg          = [ 20  -20   10  -10  5  -5]; 
amplRightLeg         = [ 20  -20   10  -10  5  -5];

freqTorso            = [ 0.1  0.1  0.1]; 
freqLeftLeg          = [ 0.1  0.1  0.1  0.1  0.1  0.1]; 
freqRightLeg         = [ 0.1  0.1  0.1  0.1  0.1  0.1]; %[ 0.0  0.3  0.0  0.0  0.0  0.0];

amplitudesOscillations   = [amplTorso,amplArms,amplArms,amplLeftLeg,amplRightLeg];
frequenciesOscillations  = [freqTorso,freqArms,freqArms,freqLeftLeg,freqRightLeg]*3;
   
if (size(amplitudesOscillations,2) ~= ROBOT_DOF) || (size(frequenciesOscillations,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable amplitudesOscillations or frequenciesOscillations. Check these variables in the file gains.m');
end