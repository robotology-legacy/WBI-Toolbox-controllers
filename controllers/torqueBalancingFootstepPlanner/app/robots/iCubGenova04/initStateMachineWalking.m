%% IF WALKING DEMO IS SELECTED, OVERWRITE SOME PARAMETERS OF COORDINATOR 
%% AND YOGA DEMO
if strcmpi(SM.SM_TYPE, 'WALKING')
    
    CONFIG.SMOOTH_DES_COM      = 0;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 0;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 0.1;
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

    gain.PCOM     =    [80    80  80  % state ==  1  TWO FEET BALANCING
                        80    80  80  % state ==  2  LEFT FOOT BALANCING
                        80    80  80];% state ==  3  RIGHT FOOT BALANCING

    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM)/10;

    gain.PAngularMomentum  = 0.25 ;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [100   100   20, 10   10    10    8, 10   10    10    8, 60   60   40    60    100 100,  60   60    40    60    100 100  % state ==  1  TWO FEET BALANCING 
                        100   100   20, 10   10    10    8, 10   10    10    8, 60   100  60   120    100 100,  60   100   60   120    100 100  % state ==  2  LEFT FOOT BALANCING
                        100   100   20, 10   10    10    8, 10   10    10    8, 60   100  60   120    100 100,  60   100   60   120    100 100]*2;% state ==  3  RIGHT FOOT BALANCING
                                          
    sm.demoOnlyBalancing             = false;
    sm.stateAt0                      = 1;
    sm.com.threshold                 = 0.05;
    sm.wrench.thresholdContactOn     = 5;      % Force threshold above which contact is considered stable
    sm.wrench.thresholdContactOff    = 60;     % Force threshold under which contact is considered off
    sm.joints.thresholdNotInContact  = 5;      % Degrees
    sm.joints.thresholdInContact     = 30;     % Degrees
    sm.tBalancing                    = 1; 
    sm.constraintsAt0                = [1; 1];

    sm.jointsSmoothingTimes          = [5;   %% state ==  1  TWO FEET BALANCING
                                        3;   %% state ==  2  LEFT FOOT BALANCING 
                                        3];  %% state ==  3  RIGHT FOOT BALANCING 

    sm.com.states      = [0.0,  0.0, 0.0;   %% state ==  1  TWO FEET BALANCING
                          0.0,  0.0, 0.0;   %% state ==  2  LEFT FOOT BALANCING 
                          0.0,  0.0, 0.0];  %% state ==  3  RIGHT FOOT BALANCING 
end

%% Walking specific parameters

% to be filled (if necessary)...
