%% IF WALKING DEMO IS SELECTED, OVERWRITE SOME OF THE PARAMETERS OF COORDINATOR 
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

    gain.footSize              = [ -0.05  0.10 ;    % xMin, xMax
                                   -0.025 0.025];   % yMin, yMax  
                   
    forceFrictionCoefficient   = 1/3;  
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling  = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics  = 0.02;

    gain.PCOM     =    [10    50  10  % state ==  1  WAITING FOR REFERENCES
                        10    50  10  % state ==  2  TWO FEET BALANCING 
                        10    50  10  % state ==  3  LEFT FOOT BALANCING
                        10    50  10];% state ==  4  RIGHT FOOT BALANCING 

    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM);

    gain.PAngularMomentum  = 0.25 ;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    % state ==  1  WAITING FOR REFERENCES
    % state ==  2  TWO FEET BALANCING 
    % state ==  3  LEFT FOOT BALANCING 
    % state ==  4  RIGHT FOOT BALANCING  

    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  1  WAITING FOR REFERENCES 
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  2  TWO FEET BALANCING
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  3  LEFT FOOT BALANCING
                        10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60     50  50, 30   50   30    60     50  50];% state ==  4  RIGHT FOOT BALANCING                                  
         
    sm.demoOnlyBalancing             = false;
    sm.com.threshold                 = 0.01;
    sm.wrench.thresholdContactOn     = 25;     % Force threshold above which contact is considered stable
    sm.wrench.thresholdContactOff    = 85;     % Force threshold under which contact is considered off
    sm.joints                        = struct;
    sm.joints.thresholdNotInContact  = 5;      % Degrees
    sm.joints.thresholdInContact     = 50;     % Degrees
    sm.stateAt0                      = 1;
    sm.tBalancing                    = 0;

    sm.jointsSmoothingTimes          = [5;   %% state ==  1  WAITING FOR REFERENCES 
                                        5;   %% state ==  2  TWO FEET BALANCING
                                        3;   %% state ==  3  LEFT FOOT BALANCING 
                                        3];  %% state ==  4  RIGHT FOOT BALANCING 

    sm.com.states      = [0.0,  0.0, 0.0;   %% state ==  1  WAITING FOR REFERENCES 
                          0.0,  0.0, 0.0;   %% state ==  2  TWO FEET BALANCING
                          0.0,  0.0, 0.0;   %% state ==  3  LEFT FOOT BALANCING 
                          0.0,  0.0, 0.0];  %% state ==  4  RIGHT FOOT BALANCING 
end

%% WALKING STATE MACHINE SPECIFIC PARAMETERS

% to be filled ...


