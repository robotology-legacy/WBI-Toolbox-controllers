%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'YOGA')
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
                   
    forceFrictionCoefficient     = 1/4;  
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling              = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics  = 0.02;

    gain.PCOM     =    [10    50  10  % state ==  1  TWO FEET BALANCING
                        10    50  10  % state ==  2  COM TRANSITION TO LEFT 
                        10    50  10  % state ==  3  LEFT FOOT BALANCING
                        10    30  30  % state ==  4  YOGA LEFT FOOT 
                        10    50  10  % state ==  5  PREPARING FOR SWITCHING 
                        10    50  10  % state ==  6  LOOKING FOR CONTACT
                        10    50  10  % state ==  7  TRANSITION TO INITIAL POSITION 
                        10    50  10  % state ==  8  COM TRANSITION TO RIGHT FOOT
                        10    50  10  % state ==  9  RIGHT FOOT BALANCING
                        10    30  30  % state == 10  YOGA RIGHT FOOT 
                        10    50  10  % state == 11  PREPARING FOR SWITCHING 
                        10    50  10  % state == 12  LOOKING FOR CONTACT
                        10    50  10];% state == 13  TRANSITION TO INITIAL POSITION
    gain.PCOM  = gain.PCOM;
    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM);

    gain.PAngularMomentum  = 0.25 ;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    % state ==  1  TWO FEET BALANCING
    % state ==  2  COM TRANSITION TO LEFT FOOT
    % state ==  3  LEFT FOOT BALANCING 
    % state ==  4  YOGA LEFT FOOT  
    % state ==  5  PREPARING FOR SWITCHING
    % state ==  6  LOOKING FOR CONTACT 
    
    % state ==  7  TRANSITION TO INITIAL POSITION
    
    % state ==  8  COM TRANSITION TO RIGHT FOOT
    % state ==  9  RIGHT FOOT BALANCING 
    % state == 10  YOGA RIGHT FOOT  
    % state == 11  PREPARING FOR SWITCHING
    % state == 12  LOOKING FOR CONTACT 
    % state == 13  TRANSITION TO INITIAL POSITION


    %                   %   TORSO  %%      LEFT ARM       %%      RIGHT ARM      %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  1  TWO FEET BALANCING
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  2  COM TRANSITION TO LEFT 
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  3  LEFT FOOT BALANCING
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  4  YOGA LEFT FOOT 
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  5  PREPARING FOR SWITCHING 
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  6  LOOKING FOR CONTACT
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  7  TRANSITION TO INITIAL POSITION 
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  8  COM TRANSITION TO RIGHT FOOT
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  9  RIGHT FOOT BALANCING
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state == 10  YOGA RIGHT FOOT 
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state == 11  PREPARING FOR SWITCHING 
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50  % state == 12  LOOKING FOR CONTACT
                        20   30   20, 12   12    12   12 10, 12   12    12   12 10, 30   50   30    60     50  50, 30   50   30    60     50  50];% state == 13  TRANSITION TO INITIAL POSITION
                    
    gain.rootPD      = [40 2*sqrt(1)];

    gain.lFoot.posPD = [40*ones(3,1),2*sqrt(ones(3,1))];
    gain.lFoot.rotPD = [40,2];

    gain.rFoot.posPD = [2*ones(3,1),2*sqrt(ones(3,1))];
    gain.rFoot.rotPD = [2,10];

    
end              
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         
gain.dampings   = sqrt(gain.impedances(1,:));


%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.skipYoga                      = false;
sm.demoOnlyRightFoot             = false;
sm.yogaAlsoOnRightFoot           = true;
sm.yogaInLoop                    = false;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     =  25;     % Force threshole above which contact is considered stable
sm.wrench.thresholdContactOff    =  85;     % Force threshole under which contact is considered off
sm.joints                        = struct;
sm.joints.thresholdNotInContact  =  5;    % Degrees
sm.joints.thresholdInContact     = 50;      % Degrees
sm.joints.pauseTimeLastPostureL  = 3;
sm.joints.pauseTimeLastPostureR  = 3;

sm.stateAt0                      = 1;

sm.DT                            = 1;
sm.waitingTimeAfterYoga          = 0;

sm.jointsSmoothingTimes          = [5;   %% state ==  1  TWO FEET BALANCING
                                         %%
                                    5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    3;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    5;   %% state ==  6  LOOKING FOR CONTACT 
                                         %%
                                    4;   %% state ==  7  TRANSITION INIT POSITION
                                         %%
                                    5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    3;   %% state ==  9  RIGHT FOOT BALANCING 
                                    4;   %% state == 10  YOGA RIGHT FOOT
                                    5;   %% state == 11  PREPARING FOR SWITCHING
                                    5;   %% state == 12  LOOKING FOR CONTACT 
                                         %%
                                    4];  %% state == 13  TRANSITION INIT POSITION

sm.com.states      = [0.0,  0.01,0.0;   %% state ==  1  TWO FEET BALANCING NOT USED
                      0.0,  0.01,0.0;     %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.0,  0.00,0.0;   %% state ==  3  LEFT FOOT BALANCING 
                      0.0,  0.01,0.0;   %% state ==  4  YOGA LEFT FOOT
                      0.0,  0.00,0.0;   %% state ==  5  PREPARING FOR SWITCHING
                      0.0, -0.09,0.0;   %% state ==  6  LOOKING FOR CONTACT 
                      0.0, -0.09,0.0;   %% state ==  7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                      % FROM NOW ON, THE REFERENCE ARE ALWAYS DELTAS W.R.T.
                      % THE POSITION OF THE RIGHT FOOT
                      0.0, -0.00,0.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.0,  0.00,0.0;   %% state ==  9  RIGHT FOOT BALANCING 
                      0.0, -0.00,0.0;   %% state == 10  YOGA RIGHT FOOT
                      0.0, -0.00,0.0;   %% state == 11  PREPARING FOR SWITCHING
                      0.0,  0.09,0.0;   %% state == 12  LOOKING FOR CONTACT 
                      0.0,  0.00,0.0];  %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
sm.tBalancing      = 0.5;%inf;%0.5;


sm.joints.states = [[0.0864,0.0258,0.0152, ...                          %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 2  COM TRANSITION TO LEFT 
                     -0.1493,0.8580,0.2437,0.8710,0 ...                   %
                     -0.1493,0.8580,0.2437,0.8710,0 ...                   %
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     %  
                    [0.0864,0.0258,0.0152, ...                          %% state == 3  LEFT FOOT BALANCING
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %    
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     % 
                    [0.0864,0.0258,0.0152, ...                          %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 5  PREPARING FOR SWITCHING
                     -0.1493,0.8580,0.2437,0.8710,0 ...                   %
                     -0.1493,0.8580,0.2437,0.8710,0 ...                   %
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051    0.0073   -0.1151];%                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 6  LOOKING FOR CONTACT
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     %   
                     zeros(1,ROBOT_DOF);                                %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                    [0.0864,0.0258,0.0152, ...                          %% state == 8  COM TRANSITION TO RIGHT FOOT
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     % 
                    [0.0864,0.0258,0.0152, ...                          %% state == 9  RIGHT FOOT BALANCING
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %    
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %  
                     zeros(1,ROBOT_DOF);                                %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                    [-0.0348,0.0779,0.0429, ...                         %% state == 11  PREPARING FOR SWITCHING
                     -0.1493,0.8580,0.2437,0.8710,0 ...                   %
                     -0.1493,0.8580,0.2437,0.8710,0 ...                   %
                      0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ... %  
                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];    %                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 12  LOOKING FOR CONTACT
                     0.1253,0.8135,0.3051,0.7928,0 ...                    %
                     0.0563,0.6789,0.3340,0.6214,0 ...                    %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277,...   %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];     %   
                    zeros(1,ROBOT_DOF)];                                %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED                     

 
q1 =        [ -0.0817   -0.3449    0.1323   ...
              -0.0613    1.3246    0.2847    0.4123   -0.3682 ...
              -0.5197    1.6034    0.5835    0.4224   -0.0398 ...
               0.2094    0.3140    0.1470   -0.1794   -0.0656    0.1609 ...
               0.3743    0.9387    1.1906   -1.6559    0.3665   -0.0727];                
                
q2 =        [ -0.0894   -0.4234    0.0855    ...
               0.0873    1.4256    0.2451    0.3049   -0.5766 ...
              -0.4224    1.6411    0.7344    0.2990   -0.2975 ...
               0.2284    0.3192    0.1540   -0.1886   -0.0795    0.2039  ...
               0.3549    1.2489    1.1961   -0.2040    0.3667   -0.0614];

q3 =        [ -0.0771   -0.1349    0.4148   ...
              -0.9720    0.7623    0.4622    0.9148   -0.4989 ...
              -0.9733    1.2994   -0.1540    0.9955   -0.1755 ...
               0.2384    0.2927    0.0833   -0.1918   -0.0569     0.1910 ...
               0.3564    1.2924    1.2023   -0.0270    0.3665    -0.0596];
          
q4 =        [ -0.0932   -0.3788    0.1309   ...
              -0.0216    1.3243    0.2751    0.3894   -0.4176 ...
              -0.5024    1.5867    0.6113    0.3941   -0.0049 ...
               0.2339    0.3040    0.1213   -0.1828   -0.0669    0.2025 ...
               0.3516    1.2909    1.1986   -0.0247    0.3670   -0.0598];
          
q5 =        [ -0.0818   -0.4317    0.0830   ...
               0.0873    1.4331    0.2444    0.3059   -0.5207 ...
              -0.4227    1.6492    0.7333    0.3004   -0.1624 ...
               0.2154    0.3602    0.1498   -0.1907   -0.0827    0.1725 ...
               0.3515    1.2871    1.2015   -1.4313    0.3665   -0.0611];
          
q6 =        [ -0.0937   -0.4164    0.0878   ...
               0.0873    1.4228    0.2445    0.3031   -0.6779 ...
              -0.4213    1.6376    0.7370    0.2964   -0.4426 ...
               0.2366    0.2996    0.1124   -0.1887   -0.0710    0.2064  ...
               0.3450    1.2914    1.2163   -0.0335    0.3665   -0.0580];
          
q7 =        [ -0.0388    0.0565    0.0448   ...
              -0.1374    0.8876    0.2424    0.8455   -0.7065 ...
              -0.1652    0.8952    0.2646    0.8450   -0.5533 ...
               0.0067   -0.0923   -0.0057   -0.0097    0.0254    0.1704  ...
               0.0166    0.1358    0.0588   -0.0058    0.0357   -0.1131];
          
q8 =        [  0.2899   -0.4496    0.4059   ...
              -0.6581    1.0421   -0.2046    0.8323   -0.8727 ...
              -0.4583    1.1892    0.0600    1.0023   -0.6166 ...
              -0.2860    0.6137   -0.6856   -0.0664   -0.2659    0.1042 ...
              -0.4645   -0.2440   -0.9310   -0.4447   -0.0603    0.3869];
          
          
  
sm.joints.pointsL =[ 0,                            q1;
                     1*sm.jointsSmoothingTimes(10),q2;
                     2*sm.jointsSmoothingTimes(10),q3;
                     3*sm.jointsSmoothingTimes(10),q4;
                     4*sm.jointsSmoothingTimes(10),q5;
                     5*sm.jointsSmoothingTimes(10),q6;
                     6*sm.jointsSmoothingTimes(10),q7;
                     7*sm.jointsSmoothingTimes(10),q8];
                 
sm.joints.pointsR = sm.joints.pointsL;

					 
for i = 1:size(sm.joints.pointsR,1)				
	sm.joints.pointsR(i,2:4)          = [sm.joints.pointsR(i,2) -sm.joints.pointsR(i,3) -sm.joints.pointsR(i,4)];
	
	rightArm                           =  sm.joints.pointsR(i,end-16:end-12);
	sm.joints.pointsR(i,end-16:end-12) =  sm.joints.pointsR(i,end-21:end-17);
	sm.joints.pointsR(i,end-21:end-17) =  rightArm;
	
	rightLeg                          =  sm.joints.pointsR(i,end-5:end);
	sm.joints.pointsR(i,end-5:end)    =  sm.joints.pointsR(i,end-11:end-6);
	sm.joints.pointsR(i,end-11:end-6) =  rightLeg;
end	 


clear q1 q2 q3 q4;
