%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM

CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                   % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                   % of the postural tasks are smoothed internally 
                                   
%Smoothing time for time varying impedances
gain.SmoothingTimeGainScheduling  = 2.5;

%% %%%%%%%%%%%%%%%%    CONTROLLER GAIN PARAMETERS

PCoM                        = [50    60    50;  % state ==  1  TWO FEET BALANCING
                               50    60    50;  % state ==  2  COM TRANSITION TO LEFT 
                               50    60    50;  % state ==  3  LEFT FOOT BALANCING
                               50    60    50;  % state ==  4  PREPARING FOR SWITCHING 
                               50    60    50;  % state ==  5  LOOKING FOR CONTACT
                               50    60    50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               50    60    50;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               50    60    50;  % state ==  8  RIGHT FOOT BALANCING
                               50    60    50;  % state ==  9  PREPARING FOR SWITCHING 
                               50    60    50;  % state == 10  LOOKING FOR CONTACT
                               50    60    50]; % state == 11  TRANSITION TO INITIAL POSITION                  

Proot                       = [5;  % state ==  1  TWO FEET BALANCING
                               5;  % state ==  2  COM TRANSITION TO LEFT 
                               5;  % state ==  3  LEFT FOOT BALANCING
                               5;  % state ==  4  PREPARING FOR SWITCHING 
                               5;  % state ==  5  LOOKING FOR CONTACT
                               5;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               5;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               5;  % state ==  8  RIGHT FOOT BALANCING
                               5;  % state ==  9  PREPARING FOR SWITCHING 
                               5;  % state == 10  LOOKING FOR CONTACT
                               5]; % state == 11  TRANSITION TO INITIAL POSITION 
                           
PlfootPos                   = [25    25    25;  % state ==  1  TWO FEET BALANCING
                               25    25    25;  % state ==  2  COM TRANSITION TO LEFT 
                               25    25    25;  % state ==  3  LEFT FOOT BALANCING
                               25    25    25;  % state ==  4  PREPARING FOR SWITCHING 
                               25    25    25;  % state ==  5  LOOKING FOR CONTACT
                               25    25    25;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               25    25    25;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               25    25    25;  % state ==  8  RIGHT FOOT BALANCING
                               25    25    25;  % state ==  9  PREPARING FOR SWITCHING 
                               25    25    25;  % state == 10  LOOKING FOR CONTACT
                               25    25    25]; % state == 11  TRANSITION TO INITIAL POSITION  
                       
PlfootRot                   = [50;  % state ==  1  TWO FEET BALANCING
                               50;  % state ==  2  COM TRANSITION TO LEFT 
                               50;  % state ==  3  LEFT FOOT BALANCING
                               50;  % state ==  4  PREPARING FOR SWITCHING 
                               50;  % state ==  5  LOOKING FOR CONTACT
                               50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               50;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               50;  % state ==  8  RIGHT FOOT BALANCING
                               50;  % state ==  9  PREPARING FOR SWITCHING 
                               50;  % state == 10  LOOKING FOR CONTACT
                               50]; % state == 11  TRANSITION TO INITIAL POSITION                             

gain.CoM.posPD              = [PCoM,      2 * sqrt(PCoM) ];
gain.root.rotPD             = [Proot,     2 * sqrt(Proot)];

gain.lFoot.posPD            = [PlfootPos, 2 * ones(11,3) ]; 
gain.lFoot.rotPD            = [PlfootRot, 2 * ones(11,1) ];
                           
gain.rFoot.posPD            = gain.lFoot.posPD;  
gain.rFoot.rotPD            = gain.lFoot.rotPD;                        

                    
%                              %  TORSO  %%     LEFT ARM     %%    RIGHT ARM     %%         LEFT LEG            %%         RIGHT LEG           %% 
gain.joints.impedances      = [20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  1  TWO FEET BALANCING
                               20   30   20, 10   10    10   10, 10   10    10   10, 25   25   15    30     50  50, 25   25   25    30     50  50;  % state ==  2  COM TRANSITION TO LEFT 
                               20   30   20, 12   12    12   12, 12   12    12   12, 25   25   15    50     50  50, 25   25   25    30     50  50;  % state ==  3  LEFT FOOT BALANCING
                               20   30   20, 10   10    10   10, 10   10    10   10, 15   25   50    30     25  25, 15   25   15    30     25  50;  % state ==  4  PREPARING FOR SWITCHING 
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  5  LOOKING FOR CONTACT
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    50     25  50, 15   25   15    50     50  50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               20   30   20, 10   10    10   10, 10   10    10   10, 25   25   25    30     50  50, 25   25   15    30     50  50;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               20   30   20, 12   12    12   12, 12   12    12   12, 50   25   25    50     50  50, 50   25   15    50     50  50;  % state ==  8  RIGHT FOOT BALANCING
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   50    30     25  25;  % state ==  9  PREPARING FOR SWITCHING 
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state == 10  LOOKING FOR CONTACT
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25]; % state == 11  TRANSITION TO INITIAL POSITION
    
gain.joints.dampings        = 2 * sqrt(gain.joints.impedances);

%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS

sm.tBalancing                    = 1;     % Time allowed for transition to initial position / waiting time for balancing on two feet
sm.tBalancingOneFoot             = 3;     % Time allowed for balancing on a single foot
sm.stateAt0                      = 1;     % Initial state
sm.demoInLoop                    = true;  % Determines if the demo is running in loop (true) or only once (false)
sm.demoOnlyRightFoot             = false; % Determines if the robot balances on right foot only (true) or on both feet (false)
sm.com.threshold                 = 0.01;  % Distance threshold under which the position of the center of mass is considered correct
sm.wrench.thresholdContactOn     = 4;     % Force threshold above which contact is considered stable
sm.wrench.thresholdContactOff    = 60;    % Force threshold under which contact is considered off
sm.joints.thresholdNotInContact  = 20;    % Degrees
sm.joints.thresholdInContact     = 20;    % Degrees
    
sm.jointsSmoothingTimes          = [1;   %% state ==  1  TWO FEET BALANCING
                                    2;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    2;   %% state ==  3  LEFT FOOT BALANCING
                                    1;   %% state ==  4  PREPARING FOR SWITCHING
                                    1;   %% state ==  5  LOOKING FOR CONTACT
                                    2;   %% state ==  6  TRANSITION INIT POSITION
                                    2;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                                    2;   %% state ==  8  RIGHT FOOT BALANCING
                                    1;   %% state ==  9  PREPARING FOR SWITCHING
                                    1;   %% state == 10  LOOKING FOR CONTACT
                                    2];  %% state == 11  TRANSITION INIT POSITION


sm.com.states       = [0.00,  0.00,   0.00;   %% state ==  1  TWO FEET BALANCING         : THIS REFERENCE IS IGNORED
                       0.00,  0.01,   0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                       0.00,  0.00,   0.00;   %% state ==  3  LEFT FOOT BALANCING
                       0.00,  0.00,   0.00;   %% state ==  4  PREPARING FOR SWITCHING
                       0.00, -0.02,  -0.01;   %% state ==  5  LOOKING FOR CONTACT
                       0.00,  0.00,   0.00;   %% state ==  6  RETURN TO INITIAL POSITION : THIS REFERENCE IS IGNORED
                       0.00, -0.01,   0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                       0.00,  0.00,   0.00;   %% state ==  8  RIGHT FOOT BALANCING 
                       0.00,  0.00,   0.00;   %% state ==  9  PREPARING FOR SWITCHING
                       0.00,  0.02,  -0.01;   %% state == 10  LOOKING FOR CONTACT
                       0.00,  0.00,   0.00];  %% state == 11  TRANSITION INIT POSITION : THIS REFERENCE IS IGNORED

                
sm.origin.leftFoot  = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING           : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  3  LEFT FOOT BALANCING          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                       0.0,  0.13,  0.05;   %% state ==  8  RIGHT FOOT BALANCING
                       0.0,  0.00, -0.01;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.13,  0.00;   %% state == 10  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                                         
sm.origin.rightFoot = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING NOT USED  : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                       0.0, -0.13,  0.05;   %% state ==  3  LEFT FOOT BALANCING
                       0.0,  0.00, -0.01;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
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
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 2  COM TRANSITION TO LEFT 
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630, ... %  
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                       [ 0.0000, 0.0000 ,0.0000, ...                         %% state == 3  LEFT FOOT BALANCING
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %    
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.0000,-0.1109, 0.0000,-0.1000, 0.0000, 0.1630, ... %  
                         0.3142, 0.0793, 0.0000,-0.7854,-0.3491,-0.1151];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 4  PREPARING FOR SWITCHING
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630, ... %
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 5  LOOKING FOR CONTACT
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.0000,-0.0741, 0.0000, 0.0000, 0.0000, 0.1369,...  %
                         0.0000, 0.0225, 0.0000, 0.0000, 0.0000,-0.0277];    % 
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 6  TRANSITION INITIAL POSITION : THIS REFERENCE IS IGNORED
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 7  COM TRANSITION TO RIGHT FOOT
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %     
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 8  RIGHT FOOT BALANCING
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.3142, 0.0793, 0.0000,-0.7854,-0.3491,-0.1151,...  %
                         0.0000,-0.1109, 0.0000,-0.1000, 0.0000, 0.1630];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 9  PREPARING FOR SWITCHING
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 10  LOOKING FOR CONTACT
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.0000, 0.0225, 0.0000, 0.0000, 0.0000,-0.0277,...  %
                         0.0000,-0.0741, 0.0000, 0.0000, 0.0000, 0.1369];    %
                       [ 0.0000, 0.0000, 0.0000, ...                         %% state == 11  TRANSITION INITIAL POSITION : THIS REFERENCE IS IGNORED
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                        -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                         0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]];   % 

                     
                     
%% %%%%%%%%%%%%%%%%    Inverse kinematics gains
ikin.CoM.posPD      = 2*gain.CoM.posPD(1,:); 

ikin.root.rotPD     = 2*gain.root.rotPD(1,:);

ikin.lFoot.posPD    = gain.lFoot.posPD(1,:);
ikin.lFoot.rotPD    = gain.lFoot.rotPD(1,:);

ikin.rFoot.posPD    = ikin.lFoot.posPD;
ikin.rFoot.rotPD    = ikin.lFoot.rotPD;

ikin.impedances     = diag([10 15 10 gain.joints.impedances(1,4:end)]);
ikin.dampings       = 2 * sqrt(ikin.impedances);


clear PCoM Proot PlfootPos PlfootRot;                  