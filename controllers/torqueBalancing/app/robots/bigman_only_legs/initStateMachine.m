%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'YOGA')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 1;
    reg.pinvDampVb             = 1e-7;
    reg.impedances             = 0.1;
    reg.dampings               = 0;
    reg.HessianQP              = 1e-7;

    sat.torque                 = 500;

    gain.footSize              = [ -0.16  0.16   ;   % xMin, xMax
                                   -0.075 0.075 ];   % yMin, yMax
                   
    forceFrictionCoefficient   = 1/3;  
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling  = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics  = 0.02;

    gain.PCOM     =    [15    20   15   % state ==  1   TWO FEET BALANCING
                        15    20   15   % state ==  2   COM TRANSITION TO LEFT 
                        15    20   15   % state ==  3   LEFT FOOT BALANCING
                        15    20   15   % state ==  4   YOGA LEFT FOOT 
                        15    20   15   % state ==  5   PREPARING FOR SWITCHING 
                        15    20   15   % state ==  6   LOOKING FOR CONTACT
                        15    20   15   % state ==  7   TRANSITION TO INITIAL POSITION 
                        15    20   15   % state ==  8   COM TRANSITION TO RIGHT FOOT
                        15    20   15   % state ==  9   RIGHT FOOT BALANCING
                        15    20   15   % state ==  10  YOGA RIGHT FOOT 
                        15    20   15   % state ==  11  PREPARING FOR SWITCHING 
                        15    20   15   % state ==  12  LOOKING FOR CONTACT
                        15    20   15]; % state ==  13  TRANSITION TO INITIAL POSITION
    
    gain.PCOM  = gain.PCOM;
    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM);
    
    gain.PAngularMomentum  = 1;
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


    %                           LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  1  TWO FEET BALANCING
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  2  COM TRANSITION TO LEFT 
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  3  LEFT FOOT BALANCING
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  4  YOGA LEFT FOOT 
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  5  PREPARING FOR SWITCHING 
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  6  LOOKING FOR CONTACT
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  7  TRANSITION TO INITIAL POSITION 
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  8  COM TRANSITION TO RIGHT FOOT
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state ==  9  RIGHT FOOT BALANCING
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state == 10  YOGA RIGHT FOOT 
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state == 11  PREPARING FOR SWITCHING 
                        30   50   30    60     50  50, 30   50   30    60     50  50  % state == 12  LOOKING FOR CONTACT
                        30   50   30    60     50  50, 30   50   30    60     50  50];% state == 13  TRANSITION TO INITIAL POSITION
                    
                    gain.impedances([4,10],:)=gain.impedances([4,10],:)/2;
end              
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         
%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.demoOnlyBalancing             = false;
sm.skipYoga                      = false;
sm.demoOnlyRightFoot             = false;
sm.yogaAlsoOnRightFoot           = true;
sm.yogaInLoop                    = false;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     = 10;          % Force threshole above which contact is considered stable
sm.wrench.thresholdContactOff    = 400;         % Force threshole under which contact is considered off
sm.joints                        = struct;
sm.joints.thresholdNotInContact  = 5;      % Degrees
sm.joints.thresholdInContact     = 50;     % Degrees
sm.joints.pauseTimeLastPostureL  = 3;
sm.joints.pauseTimeLastPostureR  = 3;

sm.stateAt0                      = 1;
sm.tBalancingOneFoot             = 1;
sm.DT                            = 1;
sm.waitingTimeAfterYoga          = 0;

sm.jointsSmoothingTimes          = [5;   %% state ==  1  TWO FEET BALANCING
                                         %%
                                    5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    3;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    1;   %% state ==  6  LOOKING FOR CONTACT 
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

sm.com.states      = [0.0,    0.0,    0.0;        %% state ==  1  TWO FEET BALANCING NOT USED
                      0.0,   -0.01,   0.0;        %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.0,   -0.01,   0.0;        %% state ==  3  LEFT FOOT BALANCING 
                      0.0,    0.0,    0.2;        %% state ==  4  YOGA LEFT FOOT
                      0.0,   -0.05,   -0.05;      %% state ==  5  PREPARING FOR SWITCHING
                      0.0,   -0.05,   -0.05;      %% state ==  6  LOOKING FOR CONTACT 
                      0.0,    0.0,    0.0;        %% state ==  7  TRANSITION INIT POSITION
                      % FROM NOW ON, THE REFERENCE ARE ALWAYS DELTAS W.R.T.
                      % THE POSITION OF THE RIGHT FOOT
                      0.0,   -0.02,    0.0;        %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.0,    0.0,    0.0;        %% state ==  9  RIGHT FOOT BALANCING 
                      0.0,    0.0,    0.2;        %% state == 10  YOGA RIGHT FOOT
                      0.0,    0.025,   -0.05;     %% state == 11  PREPARING FOR SWITCHING
                      0.0,    0.005,   -0.05;     %% state == 12  LOOKING FOR CONTACT 
                      0.0,    0.0,    0.0;];      %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
sm.tBalancing      = 0;%inf;%0.5;


sm.joints.states = [[-0.2443,0,0,0.2967,0,0 ...                           %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                     -0.2443,0,0,0.2967,0,0];                             %
                    [-0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...    %% state == 2  COM TRANSITION TO LEFT  
                      0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];      %  
                    [-0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...    %% state == 3  LEFT FOOT BALANCING
                      0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];      % 
                    [ 0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...    %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED
                      0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];      %
                    [-0.0015,-0.1909,-0.0001,-0.002,0.0160,0.1630, ...    %% state == 5  PREPARING FOR SWITCHING  
                      0.0005,0.193,-0.0014,-0.0031 0.0073 -0.1151];       %                                  
                    [ 0.0107,-0.0741,-0.0001,-0.0320,0.0252,0.1369,...    %% state == 6  LOOKING FOR CONTACT
                     -0.0026,0.0305, 0.0093,-0.0010,0.0027,-0.0277];      %   
                      zeros(1,ROBOT_DOF);                                 %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                    [ 0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...    %% state == 8  COM TRANSITION TO RIGHT FOOT
                     -0.0026,0.0305, 0.0093,-0.0010,0.0027,-0.0277];      % 
                    [ 0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...   %% state == 9  RIGHT FOOT BALANCING 
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];       %  
                     zeros(1,ROBOT_DOF);                                  %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                    [ 0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...   %% state == 11  PREPARING FOR SWITCHING 
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];       %                                  
                    [-0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277,...     %% state == 12  LOOKING FOR CONTACT
                      0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];      %   
                     zeros(1,ROBOT_DOF)];                                 %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED                     

 
q1 =        [ 0.2092,0.2960, 0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3484,0.4008,-0.0004,-0.3672,-0.0530,-0.0875];

q2 =        [ 0.2092,0.2960, 0.0006,-0.1741,-0.1044,0.0700, ... 
              0.3714,0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
          
q3 =        [ 0.2092,0.2960, 0.0006,-0.1741,-0.1044,0.0700, ...
              0.3714,0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
          
q4 =        [ 0.2092, 0.3473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q5 =        [ 0.2092, 0.3473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q6 =        [ 0.2092, 0.3473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q7 =        [ 0.2092, 0.3473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253, -1.6217, 0.6374,-0.0614];
          
q8 =        [ 0.2092, 0.3473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];

%%          
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
	
	rightLeg                          =  sm.joints.pointsR(i,end-5:end);
	sm.joints.pointsR(i,end-5:end)    =  sm.joints.pointsR(i,end-11:end-6);
	sm.joints.pointsR(i,end-11:end-6) =  rightLeg;
end	 

%% Remapping of the references in order to fit the Bigman configuration and the iCub configuration
addpath('../../../../utilityMatlabFunctions/')
numOfStates     = size(sm.joints.states,1);
ndof            = size(sm.joints.states,2);
numOfStatesYoga = 8;

for kk = 1:numOfStates
    
    sm.joints.states(kk,:)        = from_iCub_To_Bigman_JointRemapper(sm.joints.states(kk,:),ndof);
end

for kk = 1:numOfStatesYoga
    
    qjRemapped                    = from_iCub_To_Bigman_JointRemapper(sm.joints.pointsR(kk,2:end),ndof);
    sm.joints.pointsR(kk,2:end)   = qjRemapped;
    qjRemapped                    = from_iCub_To_Bigman_JointRemapper(sm.joints.pointsL(kk,2:end),ndof);
    sm.joints.pointsL(kk,2:end)   = qjRemapped;
end

q5 = from_iCub_To_Bigman_JointRemapper(q5,ndof);
q6 = from_iCub_To_Bigman_JointRemapper(q6,ndof);
q7 = from_iCub_To_Bigman_JointRemapper(q7,ndof);
q8 = from_iCub_To_Bigman_JointRemapper(q8,ndof);

clear q1 q2 q3 q4;
