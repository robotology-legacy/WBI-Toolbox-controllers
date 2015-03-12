%% icubGazeboSim

ROBOT_DOF = 1;
Kp        = 60*diag( ones(1,ROBOT_DOF)  );

ampl_and_freq = [ 20     % Amplitudes of ascillations for reference signals
                  0.1];  % Frequencies of oscillations for reference signals
% COnversion into radiants and rad/s
ampl_and_freq(1,:) = ampl_and_freq(1,:)*(pi/180);
ampl_and_freq(2,:) = ampl_and_freq(2,:)*(2*pi);
              
              
robotName = 'icub';
localName = 'impedance';
Ts        = 0.01;
Kd        = 0.0001;