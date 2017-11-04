% Configuration for internal coordinator
CONFIG.SMOOTH_DES_COM             = 0;    % If equal to one, the desired streamed values 
                                          % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q               = 0;    % If equal to one, the desired streamed values 
                                          % of the postural tasks are smoothed internally 

CONFIG.smoothingTimeTranDynamics = 0.05;
gain.SmoothingTimeGainScheduling = 0.02;

%% PARAMETERS FOR TWO FEET ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    
    gain.PCOM                 = diag([50    50  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 0*sqrt(gain.PCOM);

    gain.PAngularMomentum     = 10 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 

    impTorso            = [10   10   20
                            0    0    0]; 
    impArms             = [10   10    10    8   
                            0    0     0    0 ];                   
    impLeftLeg          = [ 30   30   30    60     10  10
                             0    0    0     0      0   0];
    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 
                         
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0];          
    intLeftLeg          = [0   0    0    0   0  0];
    intRightLeg         = [0   0    0    0   0  0];                                          
end

%% PARAMETERS FOR ONLY ONE FOOT ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 1 ;

    % Impedances acting in the null space of the desired contact forces 
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0   0];                
    intLeftLeg          = [0   0    0    0    0  0]; 
    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp; 
    impArms             = [15   15    15    8 
                            0    0     0    0]*scalingImp;                 
    impLeftLeg          = [ 30   30   30   120     10  10
                             0    0    0     0      0   0]*scalingImp; 
    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]*scalingImp; 
                               
end

sat.integral              = 0;
gain.integral             = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances           = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = 0*sqrt(gain.impedances);
gain.increasingRatesImp   = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end

gain.footSize             = [ -0.07 0.07   ; % xMin, xMax
                              -0.03 0.03 ];  % yMin, yMax   
                             
%% Regularization terms
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 0.01;
reg.pinvDampVb  = 1e-7;
reg.HessianQP   = 1e-5;
reg.impedances  = 0.1;
reg.dampings    = 0;

% CoM movements
references.com.points                  = 0;
references.com.noOscillationTime       = 0;    % that the robot waits before starting the left-and-right
references.joints.points               = 0;

forceFrictionCoefficient = 1;  

if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2 )
    
    if (CONFIG.DEMO_MOVEMENTS)
        references.com.directionOfOscillation  = [0;1;0];
        references.com.amplitudeOfOscillation  = 0.02;
        references.com.frequencyOfOscillation  = 0.2;
    else
        references.com.directionOfOscillation  = [0;0;0];
        references.com.amplitudeOfOscillation  = 0.0;  % amplitude of oscillations in meters
        references.com.frequencyOfOscillation  = 0.0;  % frequency of oscillations in hertz
    end
    
else
    
    q1 = [-0.0790    0.2279    0.4519 ...   
        -1.1621    0.6663    0.4919    0.9947  ...  
        -1.0717    1.2904   -0.2447    1.0948  ...   
         0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
         0.3484    0.4008   -0.0004   -0.3672   -0.0530   -0.0875];

    q2 = [-0.0790    0.2279    0.4519 ...
        -1.1621    0.6663    0.4965    0.9947  ...
        -1.0717    1.2904   -0.2493    1.0948  ...
         0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
         0.3714    0.1906    1.3253   -0.9794    0.6374   -0.0614 ];

    q3 = [-0.0852   -0.4273    0.0821 ...
         0.1391    1.4585    0.2464    0.3042   ... 
        -0.4181    1.6800    0.7373    0.3031  ... 
         0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
         0.3514    1.3107    1.3253   -0.0189    0.6374   -0.0614 ];

    q4 = [-0.0179  0.3218    0.0076 ...
        -0.6494  1.6296    0.0013    0.8756  ...
        -0.6524  0.8722    0.0012    0.6122  ...
         0.3850  0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
         0.2091  0.2940    0.0001   -0.1738   -0.1062    0.0781 ];

    references.joints.points = [ references.com.noOscillationTime,  q1;
                                 references.com.noOscillationTime+  references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+4*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+5*references.joints.smoothingTime, q4
                                 references.com.noOscillationTime+6*references.joints.smoothingTime, q1;
                                 references.com.noOscillationTime+7*references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+8*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+9*references.joints.smoothingTime, q4
                                 references.com.noOscillationTime+10*references.joints.smoothingTime, q1;
                                 references.com.noOscillationTime+11*references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+12*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+13*references.joints.smoothingTime, q4
                                 references.com.noOscillationTime+14*references.joints.smoothingTime, q1;
                                 references.com.noOscillationTime+15*references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+16*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+17*references.joints.smoothingTime, q4];
    clear q1 q2 q3 q4;
end

%% WORKAROUND for keeping compaibility of COORDINATOR with YOGA and WALKING controllers
sm.demoOnlyBalancing             = false;
sm.skipYoga                      = false;
sm.demoOnlyRightFoot             = false;
sm.yogaAlsoOnRightFoot           = true;
sm.yogaInLoop                    = false;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     = 25;     % Force threshold above which contact is considered stable
sm.wrench.thresholdContactOff    = 85;     % Force threshold under which contact is considered off
sm.joints                        = struct;
sm.joints.thresholdNotInContact  = 5;      % Degrees
sm.joints.thresholdInContact     = 50;     % Degrees
sm.joints.pauseTimeLastPostureL  = 3;
sm.joints.pauseTimeLastPostureR  = 3;
sm.stateAt0                      = 1;
sm.tBalancingOneFoot             = 1;
sm.waitingTimeAfterYoga          = 0;
sm.tBalancing                    = 0;

sm.com.states      = [0.0,  0.01, 0.0;   %% state ==  1  TWO FEET BALANCING NOT USED
                      0.0,  0.01, 0.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.0,  0.00, 0.0;   %% state ==  3  LEFT FOOT BALANCING 
                      0.0,  0.01, 0.0;   %% state ==  4  YOGA LEFT FOOT
                      0.0,  0.00, 0.0;   %% state ==  5  PREPARING FOR SWITCHING
                      0.0, -0.09, 0.0;   %% state ==  6  LOOKING FOR CONTACT 
                      0.0, -0.09, 0.0;   %% state ==  7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                      % FROM NOW ON, THE REFERENCE IS ALWAYS DELTAS W.R.T. THE POSITION OF THE RIGHT FOOT
                      0.0, -0.01, 0.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.0,  0.00, 0.0;   %% state ==  9  RIGHT FOOT BALANCING 
                      0.0, -0.00, 0.0;   %% state == 10  YOGA RIGHT FOOT
                      0.0, -0.00, 0.0;   %% state == 11  PREPARING FOR SWITCHING
                      0.0,  0.09, 0.0;   %% state == 12  LOOKING FOR CONTACT 
                      0.0,  0.00, 0.0];  %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED

sm.joints.states = [[0.0864,0.0258,0.0152, ...                          %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928, ...                   %
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 2  COM TRANSITION TO LEFT 
                     -0.1493,0.8580,0.2437,0.8710, ...                  %
                     -0.1493,0.8580,0.2437,0.8710, ...                  %
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     %  
                    [0.0864,0.0258,0.0152, ...                          %% state == 3  LEFT FOOT BALANCING
                     0.1253,0.8135,0.3051,0.7928, ...                   %    
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     % 
                    [0.0864,0.0258,0.0152, ...                          %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928, ...                   %
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 5  PREPARING FOR SWITCHING
                     -0.1493,0.8580,0.2437,0.8710, ...                  %
                     -0.1493,0.8580,0.2437,0.8710, ...                  %
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051    0.0073   -0.1151];%                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 6  LOOKING FOR CONTACT
                     0.1253,0.8135,0.3051,0.7928, ...                   %
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     %   
                     zeros(1,ROBOT_DOF);                                %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                    [0.0864,0.0258,0.0152, ...                          %% state == 8  COM TRANSITION TO RIGHT FOOT
                     0.1253,0.8135,0.3051,0.7928, ...                   %
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     % 
                    [0.0864,0.0258,0.0152, ...                          %% state == 9  RIGHT FOOT BALANCING
                     0.1253,0.8135,0.3051,0.7928, ...                   %    
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %  
                     zeros(1,ROBOT_DOF);                                %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                    [-0.0348,0.0779,0.0429, ...                         %% state == 11  PREPARING FOR SWITCHING
                     -0.1493,0.8580,0.2437,0.8710, ...                  %
                     -0.1493,0.8580,0.2437,0.8710, ...                  %
                      0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ... %  
                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];    %                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 12  LOOKING FOR CONTACT
                     0.1253,0.8135,0.3051,0.7928, ...                   %
                     0.0563,0.6789,0.3340,0.6214, ...                   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277,...   %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];     %   
                    zeros(1,ROBOT_DOF)];                                %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED                     

