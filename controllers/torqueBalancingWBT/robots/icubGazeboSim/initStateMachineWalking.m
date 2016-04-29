%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'WALKING')
%     reg.pinvDamp    = 0.0001;
    reg.pinvDamp = 0.001;
    sat.torque = 50;

    references.joints.smoothingTime    = 5;
    references.com.smoothingTime       = references.joints.smoothingTime;
    gain.SmoothingTimeImp              = 1;  

    smoothingTimeTransitionDynamics    = 0.02;


    gain.PCOM              = diag([50    80  50]); 
    gain.ICOM              = diag([  0    0   0]);
    gain.DCOM              = 2*sqrt(gain.PCOM);

    gain.PAngularMomentum  = 1 ;

    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [60   100   60, 10   10    10    8, 10   10    10    8, 60   50   30    60     10  10,  60   50   30    60     10  10    % state ==  1  WATING FOR REFERENCES
                        60   100   60, 10   10    10    8, 10   10    10    8, 60   60   60    90     90  10,  60  60   60    90     90  10    % state ==  2  TWO FEET BALANCING
                        10   10   20, 10   10    10    8, 10   10    10    8, 90   50   20    80     30  15,  80   50   30    80     20   5    % state ==  3  LEFT FOOT BALANCING
                        60   100   60, 30   30   30    8, 30   30    30    8, 80   50   30   100     40  25, 20   20   20  20 20  15];  % state ==  4  RIGHT FOOT BALANCING
    % Working gains when standing on right leg [130   50   20  120 60  15]
 

gain.dampings           = 0.0*sqrt(gain.impedances(4,:));
gain.dampings           = zeros(1,length(gain.impedances));
% gain.dampings(19)       = 0.001;
% gain.integral(18:23) = 0;

%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.com.threshold                 =   0.02;
sm.wrench.threshold             = -70;
sm.joints.thresholdNotInContact =  3;
sm.joints.thresholdInContact    = 110;

sm.stateAt0               = 1;

sm.DT                     = 1;

end              
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         


sm.joints.smoothingTime    = references.joints.smoothingTime;
sm.com.smoothingTime       = references.com.smoothingTime;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         