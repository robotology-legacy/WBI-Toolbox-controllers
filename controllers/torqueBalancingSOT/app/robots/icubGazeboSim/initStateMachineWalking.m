%% PARAMETERS FOR FINITE STATE MACHINE                                                                    

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
                              10;  % state ==  2  COM TRANSITION TO LEFT 
                              10;  % state ==  3  LEFT FOOT BALANCING
                               5;  % state ==  4  PREPARING FOR SWITCHING 
                              10;  % state ==  5  LOOKING FOR CONTACT
                              15;  % state ==  6  TRANSITION TO INITIAL POSITION 
                              10;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                              10;  % state ==  8  RIGHT FOOT BALANCING
                               5;  % state ==  9  PREPARING FOR SWITCHING 
                              10;  % state == 10  LOOKING FOR CONTACT
                              15]; % state == 11  TRANSITION TO INITIAL POSITION 
                           
PlfootPos                   = [20    20    20;  % state ==  1  TWO FEET BALANCING
                                0     0     0;  % state ==  2  COM TRANSITION TO LEFT 
                                0     0     0;  % state ==  3  LEFT FOOT BALANCING
                                0     0     0;  % state ==  4  PREPARING FOR SWITCHING 
                                0     0     0;  % state ==  5  LOOKING FOR CONTACT
                                0     0     0;  % state ==  6  TRANSITION TO INITIAL POSITION 
                                0     0     0;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               20    20    20;  % state ==  8  RIGHT FOOT BALANCING
                               20    20    20;  % state ==  9  PREPARING FOR SWITCHING 
                               20    20    20;  % state == 10  LOOKING FOR CONTACT
                                0     0     0]; % state == 11  TRANSITION TO INITIAL POSITION  
                       
PlfootRot                   = [5;  % state ==  1  TWO FEET BALANCING
                               5;  % state ==  2  COM TRANSITION TO LEFT 
                               5;  % state ==  3  LEFT FOOT BALANCING
                               5;  % state ==  4  PREPARING FOR SWITCHING 
                               5;  % state ==  5  LOOKING FOR CONTACT
                               5;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               5;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                              10;  % state ==  8  RIGHT FOOT BALANCING
                              10;  % state ==  9  PREPARING FOR SWITCHING 
                              10;  % state == 10  LOOKING FOR CONTACT
                               5]; % state == 11  TRANSITION TO INITIAL POSITION                             

PrfootPos                   = [20    20    20;  % state ==  1  TWO FEET BALANCING
                                0     0     0;  % state ==  2  COM TRANSITION TO LEFT 
                               20    20    20;  % state ==  3  LEFT FOOT BALANCING
                               20    20    20;  % state ==  4  PREPARING FOR SWITCHING 
                               20    20    20;  % state ==  5  LOOKING FOR CONTACT
                                0     0     0;  % state ==  6  TRANSITION TO INITIAL POSITION 
                                0     0     0;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                                0     0     0;  % state ==  8  RIGHT FOOT BALANCING
                                0     0     0;  % state ==  9  PREPARING FOR SWITCHING 
                                0     0     0;  % state == 10  LOOKING FOR CONTACT
                                0     0     0]; % state == 11  TRANSITION TO INITIAL POSITION  
                       
PrfootRot                   = [5;  % state ==  1  TWO FEET BALANCING
                               5;  % state ==  2  COM TRANSITION TO LEFT 
                              10;  % state ==  3  LEFT FOOT BALANCING
                              10;  % state ==  4  PREPARING FOR SWITCHING 
                              10;  % state ==  5  LOOKING FOR CONTACT
                               5;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               5;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               5;  % state ==  8  RIGHT FOOT BALANCING
                               5;  % state ==  9  PREPARING FOR SWITCHING 
                               5;  % state == 10  LOOKING FOR CONTACT
                               5]; % state == 11  TRANSITION TO INITIAL POSITION                             

                           
%                              %  TORSO  %%     LEFT ARM     %%    RIGHT ARM     %%         LEFT LEG            %%         RIGHT LEG           %% 
gain.joints.impedances      = [20   50   50, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  1  TWO FEET BALANCING
                               20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  2  COM TRANSITION TO LEFT 
                               20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  3  LEFT FOOT BALANCING
                               20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  4  PREPARING FOR SWITCHING 
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  5  LOOKING FOR CONTACT
                               30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  6  TRANSITION TO INITIAL POSITION 
                               20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                               20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  8  RIGHT FOOT BALANCING
                               20   20   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   15   15    15     15  15;  % state ==  9  PREPARING FOR SWITCHING 
                               20   20   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   15   15    15     15  15;  % state == 10  LOOKING FOR CONTACT
                               20   20   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   15   15    15     15  15]; % state == 11  TRANSITION TO INITIAL POSITION
   
gain.joints.dampings        = 2 * sqrt(gain.joints.impedances);

gain.CoM.posPD              = [PCoM,      2 * sqrt(PCoM) ];
gain.root.rotPD             = [Proot,     2 * sqrt(Proot)];

gain.lFoot.posPD            = [PlfootPos, 2 * sqrt(PlfootPos)]; 
gain.lFoot.rotPD            = [PlfootRot, 2 * sqrt(PlfootRot)];
                           
gain.rFoot.posPD            = [PrfootPos, 2 * sqrt(PrfootPos)]; 
gain.rFoot.rotPD            = [PrfootRot, 2 * sqrt(PrfootRot)];  

%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS

sm.tBalancing                    = 1;     % Time allowed for transition to initial position / waiting time for balancing on two feet
sm.tBalancingOneFoot             = 5;     % Time allowed for balancing on a single foot
sm.stateAt0                      = 1;     % Initial state (state 1 is TWO FEET BALANCING)
sm.demoInLoop                    = true;  % Determines if the demo is running in loop (true) or only once (false)
sm.demoOnlyRightFoot             = false; % Determines if the robot balances on right foot only (true) or on both feet (false)
sm.com.threshold                 = 0.01;  % Distance threshold under which the position of the center of mass is considered correct
sm.foot.threshold                = 0.01;  % Distance threshold under which the position of the foot is considered correct
sm.wrench.thresholdContactOn     = 4;     % Force threshold above which contact is considered stable
sm.wrench.thresholdContactOff    = 60;    % Force threshold under which contact is considered off
sm.joints.thresholdNotInContact  = 20;    % Degrees
sm.joints.thresholdInContact     = 20;    % Degrees
    
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
                       0.00,  0.01,   0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                       0.01,  0.00,   0.00;   %% state ==  3  LEFT FOOT BALANCING
                       0.01,  0.00,   0.00;   %% state ==  4  PREPARING FOR SWITCHING
                       0.01, -0.035, -0.015;   %% state ==  5  LOOKING FOR CONTACT
                       0.00,  0.00,   0.00;   %% state ==  6  RETURN TO INITIAL POSITION : THIS REFERENCE IS IGNORED
                       0.00, -0.01,   0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                       0.01,  0.00,   0.00;   %% state ==  8  RIGHT FOOT BALANCING 
                       0.01,  0.00,   0.00;   %% state ==  9  PREPARING FOR SWITCHING
                       0.01,  0.035, -0.015;   %% state == 10  LOOKING FOR CONTACT
                       0.00,  0.00,   0.00];  %% state == 11  TRANSITION INIT POSITION : THIS REFERENCE IS IGNORED

                
sm.origin.leftFoot  = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING           : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  3  LEFT FOOT BALANCING          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                       0.0,  0.13,  0.05;   %% state ==  8  RIGHT FOOT BALANCING
                       0.0,  0.00, -0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.13, -0.01;   %% state == 10  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                                         
sm.origin.rightFoot = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING NOT USED  : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                       0.0, -0.13,  0.05;   %% state ==  3  LEFT FOOT BALANCING
                       0.0,  0.00, -0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0, -0.13, -0.01;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  8  RIGHT FOOT BALANCING         : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                       0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                
sm.joints.states    = [zeros(1,ROBOT_DOF);                                   %% state == 1  TWO FEET BALANCING : THIS REFERENCE IS IGNORED
                       [-0.0350, 0.0780, 0.0430, ...                         %% state == 2  COM TRANSITION TO LEFT 
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1200, ... %  
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                       [ 0.0860, 0.0260, 0.0150, ...                         %% state == 3  LEFT FOOT BALANCING
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %    
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1200, ... %  
                         0.3142, 0.0793, 0.0000,-0.7854,-0.3491,-0.1151];    %
                       [-0.0350, 0.0780, 0.0430, ...                         %% state == 4  PREPARING FOR SWITCHING
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1200, ... %
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                       [-0.0860, 0.0260, 0.0150, ...                         %% state == 5  LOOKING FOR CONTACT
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.0000,-0.0741, 0.0000, 0.0000, 0.0250, 0.1200,...  %
                         0.0000, 0.0225, 0.0000, 0.0000, 0.0000,-0.0277];    %
                       zeros(1,ROBOT_DOF);                                   %% state == 6  TRANSITION INITIAL POSITION : THIS REFERENCE IS IGNORED
                       [ 0.0350,-0.0780, 0.0430, ...                         %% state == 7  COM TRANSITION TO RIGHT FOOT
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %     
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1200,];    %
                       [-0.0860,-0.0260, 0.0150, ...                         %% state == 8  RIGHT FOOT BALANCING
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.3142, 0.0793, 0.0000,-0.7854,-0.3491,-0.1151,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1200];    %
                       [ 0.0350,-0.0780, 0.0430, ...                         %% state == 9  PREPARING FOR SWITCHING
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                        -0.1493, 0.8580, 0.2437, 0.8710 ...                  %
                         0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                         0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1200];    %
                       [ 0.0860,-0.0260, 0.0150, ...                         %% state == 10  LOOKING FOR CONTACT
                         0.0563, 0.6789, 0.3340, 0.6214 ...                  %
                         0.1253, 0.8135, 0.3051, 0.7928 ...                  %
                         0.0000, 0.0225, 0.0000, 0.0000, 0.0000,-0.0277,...  %
                         0.0000,-0.0741, 0.0000, 0.0000, 0.0250, 0.1200];    %
                       zeros(1,ROBOT_DOF)];                                  %% state == 11  TRANSITION INITIAL POSITION : THIS REFERENCE IS IGNORED

                     
                     
%% %%%%%%%%%%%%%%%%    Inverse kinematics gains
%ikin.CoM.posPD      = 2*gain.CoM.posPD(1,:); 

%ikin.root.rotPD     = 2*gain.root.rotPD(1,:);

%ikin.lFoot.posPD    = gain.lFoot.posPD(1,:);
%ikin.lFoot.rotPD    = gain.lFoot.rotPD(1,:);

%ikin.rFoot.posPD    = ikin.lFoot.posPD;
%ikin.rFoot.rotPD    = ikin.lFoot.rotPD;

% ikin.impedances     = diag([20 30 20 gain.joints.impedances(1,4:end)]);
% ikin.dampings       = 2 * sqrt(ikin.impedances);

clear PCoM Proot PlfootPos PlfootRot PrfootPos PrfootRot;                  