%% PARAMETERS FOR FINITE STATE MACHINE                                                                    
                                   
gain.SmoothingTimeGainScheduling  = 2; %Smoothing time for time varying impedances
gain.SmoothingTimeTasks           = 2; %Smoothing time for time varying tasks
%note: keep gain.SmoothingTimeTasks <= gain.SmoothingTimeGainScheduling

%% %%%%%%%%%%%%%%%%    CONTROLLER GAIN PARAMETERS

PCoM                        = [50    60    50;  % state ==  1  TWO FEET BALANCING
                               50    60    50;  % state ==  2  COM TRANSITION TO LEFT 
                               50    60    50;  % state ==  3  LEFT FOOT BALANCING
                               50    60    50;  % state ==  4  PREPARING FOR SWITCHING 
                               60    60    50;  % state ==  5  LOOKING FOR CONTACT
                               70    60    50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               50    60    50;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               50    60    50;  % state ==  8  RIGHT FOOT BALANCING
                               50    60    50;  % state ==  9  PREPARING FOR SWITCHING 
                               60    60    50;  % state == 10  LOOKING FOR CONTACT
                               70    60    50]; % state == 11  TRANSITION TO INITIAL POSITION                  

Proot                       = [5;  % state ==  1  TWO FEET BALANCING
                               5;  % state ==  2  COM TRANSITION TO LEFT 
                              10;  % state ==  3  LEFT FOOT BALANCING
                              10;  % state ==  4  PREPARING FOR SWITCHING 
                              10;  % state ==  5  LOOKING FOR CONTACT
                               5;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               5;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                              10;  % state ==  8  RIGHT FOOT BALANCING
                              10;  % state ==  9  PREPARING FOR SWITCHING 
                              10;  % state == 10  LOOKING FOR CONTACT
                               5]; % state == 11  TRANSITION TO INITIAL POSITION 
                           
PlfootPos                   = [ 0     0     0;  % state ==  1  TWO FEET BALANCING
                                0     0     0;  % state ==  2  COM TRANSITION TO LEFT 
                                0     0     0;  % state ==  3  LEFT FOOT BALANCING
                                0     0     0;  % state ==  4  PREPARING FOR SWITCHING 
                                0     0     0;  % state ==  5  LOOKING FOR CONTACT
                                0     0     0;  % state ==  6  TRANSITION TO INITIAL POSITION 
                                0     0     0;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               90    20    70;  % state ==  8  RIGHT FOOT BALANCING
                               90    40    20;  % state ==  9  PREPARING FOR SWITCHING 
                               50    20    50;  % state == 10  LOOKING FOR CONTACT
                               30    50    30]; % state == 11  TRANSITION TO INITIAL POSITION  
                       
PlfootRot                   = [5;  % state ==  1  TWO FEET BALANCING
                               5;  % state ==  2  COM TRANSITION TO LEFT 
                               5;  % state ==  3  LEFT FOOT BALANCING
                               5;  % state ==  4  PREPARING FOR SWITCHING 
                               5;  % state ==  5  LOOKING FOR CONTACT
                               5;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               5;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                              20;  % state ==  8  RIGHT FOOT BALANCING
                              10;  % state ==  9  PREPARING FOR SWITCHING 
                              10;  % state == 10  LOOKING FOR CONTACT
                              10]; % state == 11  TRANSITION TO INITIAL POSITION                             

PrfootPos                   = [ 0     0     0;  % state ==  1  TWO FEET BALANCING
                                0     0     0;  % state ==  2  COM TRANSITION TO LEFT 
                               90    20    70;  % state ==  3  LEFT FOOT BALANCING
                               90    40    20;  % state ==  4  PREPARING FOR SWITCHING 
                               50    20    50;  % state ==  5  LOOKING FOR CONTACT
                               30    50    30;  % state ==  6  TRANSITION TO INITIAL POSITION 
                                0     0     0;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                                0     0     0;  % state ==  8  RIGHT FOOT BALANCING
                                0     0     0;  % state ==  9  PREPARING FOR SWITCHING 
                                0     0     0;  % state == 10  LOOKING FOR CONTACT
                                0     0     0]; % state == 11  TRANSITION TO INITIAL POSITION  
                       
PrfootRot                   = [5;  % state ==  1  TWO FEET BALANCING
                               5;  % state ==  2  COM TRANSITION TO LEFT 
                              20;  % state ==  3  LEFT FOOT BALANCING
                              10;  % state ==  4  PREPARING FOR SWITCHING 
                              10;  % state ==  5  LOOKING FOR CONTACT
                              10;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               5;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               5;  % state ==  8  RIGHT FOOT BALANCING
                               5;  % state ==  9  PREPARING FOR SWITCHING 
                               5;  % state == 10  LOOKING FOR CONTACT
                               5]; % state == 11  TRANSITION TO INITIAL POSITION                             

                           
%                              %  TORSO  %%     LEFT ARM     %%    RIGHT ARM     %%         LEFT LEG            %%         RIGHT LEG           %% 
gain.joints.impedances      = [40   40   40, 45   45    45   45, 45   45    45   45, 35   35   25    50     50  75, 35   35   25    50     50  75;  % state ==  1  TWO FEET BALANCING
                               40   40   40, 45   45    45   45, 45   45    45   45, 35   35   25    50    100 100, 35   35   25    50     50  75;  % state ==  2  COM TRANSITION TO LEFT 
                               40   40   40, 45   45    45   45, 45   45    45   45, 35   35   25    50    100 100, 300  35   25    200     0   0;  % state ==  3  LEFT FOOT BALANCING
                               40   40   40, 45   45    45   45, 45   45    45   45, 400  35   25   100     50  75, 0    35   25    400     0   0;  % state ==  4  PREPARING FOR SWITCHING 
                               40   40   40, 45   45    45   45, 45   45    45   45, 375  35   25    50     50  75, 300  35   25    200    50  75;  % state ==  5  LOOKING FOR CONTACT
                               40   40   40, 45   45    45   45, 45   45    45   45, 295  35   25    50     50  75, 35   35   25    200    50  75;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               40   40   40, 45   45    45   45, 45   45    45   45, 35   35   25    50     50  75, 35  100   25    50    100 100;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               40   40   40, 45   45    45   45, 45   45    45   45  300  35   25    400    0    0, 35  100   25    50    100 100;  % state ==  8  RIGHT FOOT BALANCING
                               40   40   40, 45   45    45   45, 45   45    45   45, 0    35   25    600    0    0, 300  35   25    50     50  75;  % state ==  9  PREPARING FOR SWITCHING 
                               40   40   40, 45   45    45   45, 45   45    45   45, 300  35   25    600    50  75, 300  35   25    50     50  75;  % state == 10  LOOKING FOR CONTACT
                               40   40   40, 45   45    45   45, 45   45    45   45, 35   35   25    50     50  75, 50   35   25    50     50  75]; % state == 11  TRANSITION TO INITIAL POSITION

gain.joints.dampings        = 2 * sqrt(gain.joints.impedances) * 0;

gain.CoM.posPD              = [PCoM,      2 * sqrt(PCoM) /40];
gain.root.rotPD             = [Proot,     2 * sqrt(Proot)/40];

gain.lFoot.posPD            = [PlfootPos, 2 * sqrt(PlfootPos)/40]; 
gain.lFoot.rotPD            = [PlfootRot, 2 * sqrt(PlfootRot)/40];
                           
gain.rFoot.posPD            = [PrfootPos, 2 * sqrt(PrfootPos)/40]; 
gain.rFoot.rotPD            = [PrfootRot, 2 * sqrt(PrfootRot)/40];  

%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS

sm.tBalancing                    = 5;    % Time allowed for transition to initial position / waiting time for balancing on two feet
sm.tBalancingOneFoot             = 15;     % Time allowed for balancing on a single foot
sm.stateAt0                      = 6;     % Initial state
sm.demoInLoop                    = true;  % Determines if the demo is running in loop (true) or only once (false)
sm.demoOnlyRightFoot             = false; % Determines if the robot balances on right foot only (true) or on both feet (false)
sm.com.threshold                 = 0.013;  % Distance threshold under which the position of the center of mass is considered correct
sm.wrench.thresholdContactOn     = 14; %20;     % Force threshold above which contact is considered stable
sm.wrench.thresholdContactOff    = 70;    % Force threshold under which contact is considered off
sm.joints.thresholdNotInContact  = 24; %30    % Degrees
sm.joints.thresholdInContact     = 24; %20    % Degrees
    
sm.jointsSmoothingTimes          = [2;   %% state ==  1  TWO FEET BALANCING
                                    2;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    2;   %% state ==  3  LEFT FOOT BALANCING
                                    2;   %% state ==  4  PREPARING FOR SWITCHING
                                    2;   %% state ==  5  LOOKING FOR CONTACT
                                    2;   %% state ==  6  TRANSITION INIT POSITION
                                    2;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                                    2;   %% state ==  8  RIGHT FOOT BALANCING
                                    2;   %% state ==  9  PREPARING FOR SWITCHING
                                    2;   %% state == 10  LOOKING FOR CONTACT
                                    2];  %% state == 11  TRANSITION INIT POSITION


sm.com.states       = [0.00,  0.00,   0.00;   %% state ==  1  TWO FEET BALANCING         : THIS REFERENCE IS IGNORED
                       0.01,  0.00,  -0.005;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                       0.01,  0.00,  -0.005;   %% state ==  3  LEFT FOOT BALANCING
                       0.01,  0.00,  -0.005;   %% state ==  4  PREPARING FOR SWITCHING
                       0.01, -0.02,  -0.01;   %% state ==  5  LOOKING FOR CONTACT
                       0.00,  0.00,   0.00;   %% state ==  6  RETURN TO INITIAL POSITION : THIS REFERENCE IS IGNORED
                       0.01,  0.005*0, -0.005;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                       0.01,  0.005*0, -0.005;   %% state ==  8  RIGHT FOOT BALANCING 
                       0.01,  0.005*0, -0.005;   %% state ==  9  PREPARING FOR SWITCHING
                       0.01,  0.005*4, -0.01;   %% state == 10  LOOKING FOR CONTACT
                       0.00,  0.00,   0.00];  %% state == 11  TRANSITION INIT POSITION : THIS REFERENCE IS IGNORED

                
sm.origin.leftFoot  = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING           : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  3  LEFT FOOT BALANCING          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                       0.0,  0.13,  0.05;   %% state ==  8  RIGHT FOOT BALANCING
                       0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.13,  0.00;   %% state == 10  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                                         
sm.origin.rightFoot = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING NOT USED  : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                       0.0, -0.13,  0.05;   %% state ==  3  LEFT FOOT BALANCING
                       0.0,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0, -0.13,  0.00;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  8  RIGHT FOOT BALANCING         : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                   
sm.joints.states    = [[ 0.0000, 0.0000, 0.0000, ...                         %% state == 1  TWO FEET BALANCING : THIS REFERENCE IS IGNORED
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, ... %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000];    %
                       [-0.0350, 0.0780, 0.0430, ...                         %% state == 2  COM TRANSITION TO LEFT 
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1815, ... %  
                         0.0000, 0.0793, 0.0000,-0.1000, 0.0000,-0.1151];    %
                       [ 0.0860, 0.0260, 0.0150, ...                         %% state == 3  LEFT FOOT BALANCING
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %    
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1815, ... %  
                         0.3142, 0.0793, 0.0000,-0.7854,-0.3491,-0.1151];    %
                       [-0.0350, 0.0780, 0.0430, ...                         %% state == 4  PREPARING FOR SWITCHING
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1815, ... %
                         0.0000, 0.0793, 0.0000,-0.1000, 0.0000,-0.1151];    %
                       [-0.0860, 0.0260, 0.0150, ...                         %% state == 5  LOOKING FOR CONTACT
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.0000,-0.0741, 0.0000, 0.0000, 0.0250, 0.1815,...  %
                         0.0000, 0.0225, 0.0000,-0.1000, 0.0000,-0.0277];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 6  TRANSITION INITIAL POSITION : THIS REFERENCE IS IGNORED
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000];    %
                       [ 0.0350,-0.0780, 0.0430, ...                         %% state == 7  COM TRANSITION TO RIGHT FOOT
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %     
                         0.0000, 0.0793, 0.0000,-0.1000, 0.0000,-0.1151*0,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1815,];    %
                       [-0.0860,-0.0260, 0.0150, ...                         %% state == 8  RIGHT FOOT BALANCING
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.3142, 0.0793, 0.0000,-0.7854,-0.3491,-0.1151*0,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1815];    %
                       [ 0.0350,-0.0780, 0.0430, ...                         %% state == 9  PREPARING FOR SWITCHING
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000, 0.0793, 0.0000,-0.1000, 0.0000,-0.1151*0,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1815];    %
                       [ 0.0860,-0.0260, 0.0150, ...                         %% state == 10  LOOKING FOR CONTACT
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.0000, 0.0225, 0.0000,-0.1000, 0.0000,-0.0277*0,...  %
                         0.0000,-0.0741, 0.0000, 0.0000, 0.0250, 0.1815];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 11  TRANSITION INITIAL POSITION : THIS REFERENCE IS IGNORED
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]];   % 
                     
%% %%%%%%%%%%%%%%%%    Inverse kinematics gains
%ikin.CoM.posPD      = [50 * ones(3, 1)', 2*sqrt(50*ones(3,1))'];  %2*gain.CoM.posPD(1,:); 

%ikin.root.rotPD     = [5, 2*sqrt(5)]; %2*gain.root.rotPD(1,:);

%ikin.lFoot.posPD    = [25*ones(1,3), 2*sqrt(25*ones(1,3))];
%ikin.lFoot.rotPD    = [50, 2*sqrt(50)];

%ikin.rFoot.posPD    = ikin.lFoot.posPD;
%ikin.rFoot.rotPD    = ikin.lFoot.rotPD;

%ikin.impedances     = diag(gain.joints.impedances(1,:));
%ikin.dampings       = 2 * sqrt(ikin.impedances);

clear PCoM Proot PlfootPos PlfootRot PrfootPos PrfootRot; 