%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM

CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                   % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                   % of the postural tasks are smoothed internally 
                                   
%Smoothing time for time varying impedances
gain.SmoothingTimeGainScheduling  = 2.5;

%% %%%%%%%%%%%%%%%%    CONTROLLER GAIN PARAMETERS

gain.PCOM            = [50    60  50;  % state ==  1  TWO FEET BALANCING
                        50    60  50;  % state ==  2  COM TRANSITION TO LEFT 
                        50    60  50;  % state ==  3  LEFT FOOT BALANCING
                        50    60  50;  % state ==  4  PREPARING FOR SWITCHING 
                        50    60  50;  % state ==  5  LOOKING FOR CONTACT
                        50    60  50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                        50    60  50;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                        50    60  50;  % state ==  8  RIGHT FOOT BALANCING
                        50    60  50;  % state ==  9  PREPARING FOR SWITCHING 
                        50    60  50;  % state == 10  LOOKING FOR CONTACT
                        50    60  50]; % state == 11  TRANSITION TO INITIAL POSITION

gain.DCOM            = 2 * sqrt(gain.PCOM);                    
                    
%                    %  TORSO  %%    LEFT ARM     %%    RIGHT ARM     %%         LEFT LEG            %%         RIGHT LEG           %% 
gain.impedances      = [20   30   20, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  1  TWO FEET BALANCING
                        20   30   20, 10   10    10   10, 10   10    10   10, 25   25   15    30     50  50, 25   25   25    30     50  50;  % state ==  2  COM TRANSITION TO LEFT 
                        20   30   20, 12   12    12   12, 12   12    12   12, 25   25   15    50     50  50, 25   25   25    30     50  50;  % state ==  3  LEFT FOOT BALANCING
                        30   30   30, 10   10    10   10, 10   10    10   10, 15   25   50    30     25  25, 15   25   15    30     25  50;  % state ==  4  PREPARING FOR SWITCHING 
                        30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state ==  5  LOOKING FOR CONTACT
                        30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    50     25  50, 15   25   15    50     50  50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                        20   30   20, 10   10    10   10, 10   10    10   10, 25   25   25    30     50  50, 25   25   15    30     50  50;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                        20   30   20, 12   12    12   12, 12   12    12   12, 50   25   25    50     50  50, 50   25   15    50     50  50;  % state ==  8  RIGHT FOOT BALANCING
                        30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   50    30     25  25;  % state ==  9  PREPARING FOR SWITCHING 
                        30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25;  % state == 10  LOOKING FOR CONTACT
                        30   30   30, 10   10    10   10, 10   10    10   10, 15   25   15    30     25  25, 15   25   15    30     25  25]; % state == 11  TRANSITION TO INITIAL POSITION
    
gain.dampings          = 2 * sqrt(gain.impedances);

%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS

sm.tBalancing                    = 1;     %Time allowed for transition to initial position / waiting time for balancing on two feet
sm.DT                            = 5;     %Time allowed for balancing on a single foot
sm.stateAt0                      = 1;     %Initial state
sm.demoInLoop                    = true;
sm.demoOnlyRightFoot             = false;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     = 4;     % Force threshold above which contact is considered stable
sm.wrench.thresholdContactOff    = 100;   % Force threshold under which contact is considered off
sm.joints.thresholdNotInContact  = 35;    % Degrees
sm.joints.thresholdInContact     = 50;    % Degrees
    
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


sm.com.states      = [0.00,  0.00,   0.00;   %% state ==  1  TWO FEET BALANCING NOT USED
                      0.00,  0.00,   0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.00,  0.00,   0.00;   %% state ==  3  LEFT FOOT BALANCING
                      0.00,  0.00,   0.00;   %% state ==  4  PREPARING FOR SWITCHING
                      0.00, -0.02,  -0.01;   %% state ==  5  LOOKING FOR CONTACT
                      0.00,  0.00,   0.00;   %% state ==  6  RETURN TO INITIAL POSITION
                      0.00,  0.00,   0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                      0.00,  0.00,   0.00;   %% state ==  8  RIGHT FOOT BALANCING 
                      0.00,  0.00,   0.00;   %% state ==  9  PREPARING FOR SWITCHING
                      0.00,  0.02,  -0.01;   %% state == 10  LOOKING FOR CONTACT
                      0.00,  0.00,   0.00];  %% state == 11  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED

                
sm.origin.leftFoot = [0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING NOT USED  : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  3  LEFT FOOT BALANCING          : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                      0.0,  0.13,  0.05;   %% state ==  8  RIGHT FOOT BALANCING
                      0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0,  0.13, -0.02;   %% state == 10  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                                         
sm.origin.rightFoot =[0.0,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING NOT USED  : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT  : THIS REFERENCE IS IGNORED
                      0.0, -0.13,  0.05;   %% state ==  3  LEFT FOOT BALANCING
                      0.0,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0, -0.13, -0.02;   %% state ==  5  LOOKING FOR CONTACT          : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION   : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  8  RIGHT FOOT BALANCING         : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                
sm.joints.states = [[ 0.0000, 0.0000, 0.0000, ...                         %% state == 1  TWO FEET BALANCING
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
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
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
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 6  TRANSITION INITIAL POSITION
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
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
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
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 6  TRANSITION INITIAL POSITION
                     -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                     -0.5131, 0.5306, 0.0000, 0.7782 ...                  %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]];   % 
                  