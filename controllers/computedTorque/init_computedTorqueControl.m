%% icubGazeboSim

ROBOT_DOF = 3;
Kp        = 260*diag( ones(1,ROBOT_DOF)  );

ampl_and_freq = [ 20     % Amplitudes of ascillations for reference signals
                  0.4];  % Frequencies of oscillations for reference signals
              
              
simulationTime = 5/ampl_and_freq(2);               
% COnversion into radiants and rad/s
ampl_and_freq(1,:) = ampl_and_freq(1,:)*(pi/180);
ampl_and_freq(2,:) = ampl_and_freq(2,:)*(2*pi);
              
              
robotName = 'icub';
localName = 'computedTorque';
Ts        = 0.01;
Kd        = 2*sqrt(Kp);