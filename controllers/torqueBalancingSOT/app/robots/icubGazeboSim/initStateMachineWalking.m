%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'WALKING')
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 1;
    reg.impedances             = 0.1;
    reg.dampings               = 0;
    reg.HessianQP              = 1e-4;
    reg.jointAnglesQP          = 0;
    reg.torquesQP              = 1e-3;
    reg.taskAccQP              = 5e-3;

    sat.torque                 = 60;    
    sat.torqueDot              = 100*ones(ROBOT_DOF,1);

    gain.footSize              = [ -0.05  0.10 ;    % xMin, xMax
                                   -0.025 0.025];   % yMin, yMax  
                   
    forceFrictionCoefficient   = 1/4;  
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling  = 2.5; %2;  

    %Smoothing time for time-varying constraints
   CONFIG.smoothingTimeTranDynamics  = 0.02;

    gain.PCOM     =    [50    60  50;  % state ==  1  TWO FEET BALANCING
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
                    
    gain.PCOM  = gain.PCOM;
    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM);

    gain.PAngularMomentum  = 0.25 ;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    % state ==  1  TWO FEET BALANCING
    % state ==  2  COM TRANSITION TO LEFT FOOT
    % state ==  3  LEFT FOOT BALANCING 
    % state ==  4  PREPARING FOR SWITCHING
    % state ==  5  LOOKING FOR CONTACT 
    
    % state ==  6  TRANSITION TO INITIAL POSITION
    
    % state ==  7  COM TRANSITION TO RIGHT FOOT
    % state ==  8  RIGHT FOOT BALANCING 
    % state ==  9  PREPARING FOR SWITCHING
    % state == 10  LOOKING FOR CONTACT 
    % state == 11  TRANSITION TO INITIAL POSITION


    %                   %   TORSO  %%      LEFT ARM       %%      RIGHT ARM      %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30    60   100  100,  30   50   30    60   100  100;  % state ==  1  TWO FEET BALANCING
                        20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30    60   100  100,  30   50   30    60   100  100;  % state ==  2  COM TRANSITION TO LEFT 
                        20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30    60   100  100,  30   50   30    60   100  100;  % state ==  3  LEFT FOOT BALANCING
                        30   30   30, 12   12   12   12   10, 12   12   12   12   10, 200  250   30    60    50   50, 220  550  220   200    65  300;  % state ==  4  PREPARING FOR SWITCHING 
                        90   90   90, 12   12   12   12   10, 12   12   12   12   10, 100  350   30   200    50  100, 220  550  220   200    65  300;  % state ==  5  LOOKING FOR CONTACT
                        20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30   100    50   50,  30   50   30   100    50   50;  % state ==  6  TRANSITION TO INITIAL POSITION 
                        20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30    60   100  100,  30   50   30    60   100  100;  % state ==  7  COM TRANSITION TO RIGHT FOOT
                        20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30    60   100  100,  30   50   30    60   100  100;  % state ==  8  RIGHT FOOT BALANCING
                        30   30   30, 12   12   12   12   10, 12   12   12   12   10, 220  550  220   200    65  300, 200  250   30    60    50   50;  % state ==  9  PREPARING FOR SWITCHING 
                        90   90   90, 12   12   12   12   10, 12   12   12   12   10, 220  550  220   200    65  300, 100  350   30   200   100  100;  % state == 10  LOOKING FOR CONTACT
                        20   30   20, 12   12   12   12   10, 12   12   12   12   10,  30   50   30    90    50   50,  30   50   30    90    50   50]; % state == 11  TRANSITION TO INITIAL POSITION

   
    gain.rootPD      = [5 2];

    gain.lFoot.posPD = [25*ones(3,1),2*sqrt(ones(3,1))];
    gain.lFoot.rotPD = [25,2];

    gain.rFoot.posPD = [25*ones(3,1),2*sqrt(ones(3,1))];
    gain.rFoot.rotPD = [25,2];

    
end              
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         
gain.weightPostural         = 0.5; %0.15;
gain.weightTasks            = 100;
gain.impedances(:,14:end)   = gain.impedances(:,14:end)/3; %/5
gain.dampings               = sqrt(gain.impedances(1,:));

%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.skipYoga                      = true;
sm.demoOnlyRightFoot             = false;
sm.yogaAlsoOnRightFoot           = true;
sm.yogaInLoop                    = true;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     = 10;    % Force threshold above which contact is considered stable
sm.wrench.thresholdContactOff    = 100;   % Force threshold under which contact is considered off
sm.joints                        = struct;
sm.joints.thresholdNotInContact  = 35;    % Degrees
sm.joints.thresholdInContact     = 50;    % Degrees
sm.joints.pauseTimeLastPostureL  = 3;
sm.joints.pauseTimeLastPostureR  = 3;

sm.stateAt0                      = 1;

sm.DT                            = 5;
sm.waitingTimeAfterYoga          = 0;

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


sm.com.states      = [0.00,  0.00,  0.00;   %% state ==  1  TWO FEET BALANCING NOT USED
                      0.00,  0.00,  0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.00,  0.00,  0.00;   %% state ==  3  LEFT FOOT BALANCING
                      0.00,  0.00,  0.00;   %% state ==  4  PREPARING FOR SWITCHING
                      0.00, -0.015,-0.01;   %% state ==  5  LOOKING FOR CONTACT
                      0.00,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION
                      0.00,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT
                      0.00,  0.00,  0.00;   %% state ==  8  RIGHT FOOT BALANCING 
                      0.00,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING
                      0.00,  0.015,-0.01;   %% state == 10  LOOKING FOR CONTACT
                      0.00,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED

                
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
                      0.0,  0.00,  0.00;   %% state ==  6  RETURN TO INITIAL POSITION    : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  7  COM TRANSITION TO RIGHT FOOT : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  8  RIGHT FOOT BALANCING         : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00;   %% state ==  9  PREPARING FOR SWITCHING      : THIS REFERENCE IS IGNORED
                      0.0,  0.00,  0.00];  %% state == 11  TRANSITION INIT POSITION     : THIS REFERENCE IS IGNORED
                  
                 
sm.tBalancing      = 1; %inf;%0.5;

sm.joints.states = [[ 0.0000, 0.0000, 0.0000, ...                         %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                     -0.5131, 0.5306, 0.0000, 0.7782, 0 ...               %
                     -0.5131, 0.5306, 0.0000, 0.7782, 0 ...               %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, ... %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 2  COM TRANSITION TO LEFT 
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                      0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630, ... %  
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                    [ 0.0000, 0.0000 ,0.0000, ...                         %% state == 3  LEFT FOOT BALANCING
                      0.1253, 0.8135, 0.3051, 0.7928, 0 ...               %    
                      0.0563, 0.6789, 0.3340, 0.6214, 0 ...               %
                      0.0000,-0.1109, 0.0000,-0.1000, 0.0000, 0.1630, ... %  
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 4  PREPARING FOR SWITCHING
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                      0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630, ... %
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 5  LOOKING FOR CONTACT
                      0.1253, 0.8135, 0.3051, 0.7928, 0 ...               %
                      0.0563, 0.6789, 0.3340, 0.6214, 0 ...               %
                      0.0000,-0.0741, 0.0000, 0.0000, 0.0000, 0.1369,...  %
                      0.0000, 0.0225, 0.0000, 0.0000, 0.0000,-0.0277];    % 
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 6  TRANSITION INITIAL POSITION
                     -0.5131, 0.5306, 0.0000, 0.7782, 0 ...               %
                     -0.5131, 0.5306, 0.0000, 0.7782, 0 ...               %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 7  COM TRANSITION TO RIGHT FOOT
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %     
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                      0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 8  RIGHT FOOT BALANCING
                      0.0563, 0.6789, 0.3340, 0.6214, 0 ...               %
                      0.1253, 0.8135, 0.3051, 0.7928, 0 ...               %
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                      0.0000,-0.1109, 0.0000,-0.1000, 0.0000, 0.1630];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 9  PREPARING FOR SWITCHING
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                     -0.1493, 0.8580, 0.2437, 0.8710, 0 ...               %
                      0.0000, 0.0793, 0.0000, 0.0000, 0.0000,-0.1151,...  %
                      0.0000,-0.1109, 0.0000, 0.0000, 0.0000, 0.1630];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 10  LOOKING FOR CONTACT
                      0.0563, 0.6789, 0.3340, 0.6214, 0 ...               %
                      0.1253, 0.8135, 0.3051, 0.7928, 0 ...               %
                      0.0000, 0.0225, 0.0000, 0.0000, 0.0000,-0.0277,...  %
                      0.0000,-0.0741, 0.0000, 0.0000, 0.0000, 0.1369];    %
                    [ 0.0000, 0.0000, 0.0000, ...                         %% state == 6  TRANSITION INITIAL POSITION
                     -0.5131, 0.5306, 0.0000, 0.7782, 0 ...               %
                     -0.5131, 0.5306, 0.0000, 0.7782, 0 ...               %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,...  %
                      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]];   % 

% % sm.joints.states = [[ 0.0864,0.0258,0.0152, ...                         %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %
% %                       0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...  %
% %                       0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];    %
% %                     [-0.0348,0.0779,0.0429, ...                         %% state == 2  COM TRANSITION TO LEFT 
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
% %                       0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];    %
% %                     [ 0.0864,0.0258,0.0152, ...                         %% state == 3  LEFT FOOT BALANCING
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %    
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %
% %                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
% %                       0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];    %
% %                     [-0.0348,0.0779,0.0429, ...                         %% state == 4  PREPARING FOR SWITCHING
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %
% %                       0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];    %
% %                     [ 0.0864,0.0258,0.0152, ...                         %% state == 5  LOOKING FOR CONTACT
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %
% %                       0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...  %
% %                      -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     % 
% %                     [ 0.0864,0.0258,0.0152, ...                         %% state == 6  TRANSITION INITIAL POSITION
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %
% %                       0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...  %
% %                       0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];    %
% %                     [-0.0348,0.0779,0.0429, ...                         %% state == 7  COM TRANSITION TO RIGHT FOOT
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                       0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151,...  %
% %                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %
% %                     [ 0.0864,0.0258,0.0152, ...                         %% state == 8  RIGHT FOOT BALANCING
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %    
% %                       0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151,...  %  
% %                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %
% %                     [-0.0348,0.0779,0.0429, ...                         %% state == 9  PREPARING FOR SWITCHING
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                      -0.1493,0.8580,0.2437,0.8710,0 ...                 %
% %                       0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ... %  
% %                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %
% %                     [ 0.0864,0.0258,0.0152, ...                         %% state == 10  LOOKING FOR CONTACT
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %
% %                      -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277,...   %
% %                       0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];    %
% %                     [ 0.0864,0.0258,0.0152, ...                         %% state == 11  TRANSITION INITIAL POSITION
% %                       0.1253,0.8135,0.3051,0.7928,0 ...                 %
% %                       0.0563,0.6789,0.3340,0.6214,0 ...                 %
% %                       0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...  %
% %                       0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931]];   %
% %                  
