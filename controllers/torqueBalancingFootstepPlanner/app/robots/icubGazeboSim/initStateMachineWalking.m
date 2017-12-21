%% IF WALKING DEMO IS SELECTED, OVERWRITE SOME OF THE PARAMETERS OF COORDINATOR 
%% AND YOGA DEMO 
if strcmpi(SM.SM_TYPE, 'WALKING')
    
    CONFIG.SMOOTH_DES_COM      = 0;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 0;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 1;
    reg.impedances             = 0.1;
    reg.dampings               = 0;
    reg.HessianQP              = 1e-7;

    sat.torque                 = 60;

    gain.footSize              = [-0.05  0.10 ;    % xMin, xMax
                                  -0.025 0.025];   % yMin, yMax  
                   
    forceFrictionCoefficient   = 1/3;  
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling  = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics  = 0.02;

    gain.PCOM     =    [30    30  10  % state ==  1  TWO FEET BALANCING 
                        40    40  40  % state ==  2  LEFT FOOT BALANCING
                        40    40  40];% state ==  3  RIGHT FOOT BALANCING 

    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM)/10;

    gain.PAngularMomentum  = 0.25;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    % state ==  1  TWO FEET BALANCING 
    % state ==  2  LEFT FOOT BALANCING 
    % state ==  3  RIGHT FOOT BALANCING  

    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  1  TWO FEET BALANCING
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  2  LEFT FOOT BALANCING
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50];% state ==  3  RIGHT FOOT BALANCING                                  
         
    sm.demoOnlyBalancing             = false;
    sm.com.threshold                 = 0.01;
    sm.wrench.thresholdContactOn     = 5;     % Force threshold above which contact is considered stable
    sm.wrench.thresholdContactOff    = 75;     % Force threshold under which contact is considered off
    sm.joints.thresholdNotInContact  = 5;      % Degrees
    sm.joints.thresholdInContact     = 50;     % Degrees
    sm.stateAt0                      = 1;
    sm.constraintsAt0                = [1; 1];
    sm.tBalancing                    = 0.5;

    sm.jointsSmoothingTimes          = [5;   %% state ==  1  TWO FEET BALANCING
                                        3;   %% state ==  2  LEFT FOOT BALANCING 
                                        3];  %% state ==  3  RIGHT FOOT BALANCING 

    sm.com.states      = [0.0,  0.0, 0.0;   %% state ==  1  TWO FEET BALANCING
                          0.0,  0.0, 0.0;   %% state ==  2  LEFT FOOT BALANCING 
                          0.0,  0.0, 0.0];  %% state ==  3  RIGHT FOOT BALANCING 
end

%% WALKING STATE MACHINE SPECIFIC PARAMETERS

% to be filled (if necessary)...


