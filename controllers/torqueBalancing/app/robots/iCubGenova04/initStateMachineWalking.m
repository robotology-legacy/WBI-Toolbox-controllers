%% IF WALKING DEMO IS SELECTED, OVERWRITE SOME PARAMETERS OF COORDINATOR 
%% AND YOGA DEMO
if strcmpi(SM.SM_TYPE, 'WALKING')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 1;
    reg.impedances             = 0.1;
    reg.dampings               = 0;
    reg.HessianQP              = 1e-7;

    sat.torque                 = 60;

    gain.footSize              = [ -0.07  0.12 ;    % xMin, xMax
                                   -0.045 0.05 ];   % yMin, yMax  
                   
    forceFrictionCoefficient   = 1/3;  
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics = 0.02;

    gain.PCOM     =    [50    50  10  % state ==  1  WAITING FOR REFERENCES
                        50    50  10  % state ==  2  TWO FEET BALANCING
                        50    50  10  % state ==  3  LEFT FOOT BALANCING
                        50    50  10];% state ==  4  YOGA LEFT FOOT 

    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM)/20;

    gain.PAngularMomentum  = 0.25 ;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   30   20    20    100 100,  30   50   30    60    100 100  % state ==  1  WAITING FOR REFERENCES
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   30   20    20    100 100,  30   50   30    60    100 100  % state ==  2  TWO FEET BALANCING 
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    100 100,  30   30   20    20    100 100  % state ==  3  LEFT FOOT BALANCING
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    100 100,  30   30   20    20    100 100];% state ==  4  RIGHT FOOT BALANCING
                                          
    sm.demoOnlyBalancing             = false;
    sm.stateAt0                      = 1;
    sm.com.threshold                 = 0.01;
    sm.wrench.thresholdContactOn     = 50;     % Force threshold above which contact is considered stable
    sm.wrench.thresholdContactOff    = 100;    % Force threshold under which contact is considered off
    sm.joints                        = struct;
    sm.joints.thresholdNotInContact  = 5;      % Degrees
    sm.joints.thresholdInContact     = 30;     % Degrees
    sm.tBalancing                    = 1; 

    sm.jointsSmoothingTimes          = [1;   %% state ==  1  WAITING FOR REFERENCES
                                        1;   %% state ==  2  TWO FEET BALANCING
                                        1;   %% state ==  3  LEFT FOOT BALANCING 
                                        1];  %% state ==  4  RIGHT FOOT BALANCING 

    sm.com.states      = [0.0,  0.01, 0.0;   %% state ==  1  WAITING FOR REFERENCES
                          0.0,  0.00, 0.0;   %% state ==  2  TWO FEET BALANCING
                          0.0,  0.005,0.0;   %% state ==  3  LEFT FOOT BALANCING 
                          0.0,  0.00, 0.0];  %% state ==  4  RIGHT FOOT BALANCING 
end

% walking specific parameters (to be filled ...)