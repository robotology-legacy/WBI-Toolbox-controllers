%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if  strcmpi(SM.SM_TYPE, 'YOGA')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 1;
    reg.pinvDampVb             = 1e-7;
    reg.impedances             = 0.1;
    reg.dampings               = 0;
    reg.HessianQP              = 1e-7;

    sat.torque                 = 5000;
    forceFrictionCoefficient   = 1/3;  
    gain.footSize              = [-0.16   0.16   ;   % xMin, xMax
                                  -0.075  0.075 ];   % yMin, yMax
                   
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling  = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics  = 0.02;

    gain.PCOM     =    [15    20   15   % state ==  1   TWO FEET BALANCING
                        25    35   25   % state ==  2   COM TRANSITION TO LEFT 
                        25    35   25   % state ==  3   LEFT FOOT BALANCING
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
    
    gain.PAngularMomentum  = 0.95;
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

                       %   TORSO  %%        LEFT ARM   %%            RIGHT ARM   %%             LEFT LEG            %%        RIGHT LEG           %% 
    gain.impedances  = [10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  1  TWO FEET BALANCING
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  2  COM TRANSITION TO LEFT 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  3  LEFT FOOT BALANCING
                        80  120   80,  40   40    40   40   40,  40   40    40   40   40,  80   80  250   200   50  50,  70   70   70    70   50  50   % state ==  4  YOGA LEFT FOOT 
                        30   30   30,  10   10    10    8   10,  10   10    10    8   10,  30   50  300    60   50  50,  30   50   30    60   50  50   % state ==  5  PREPARING FOR SWITCHING 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  6  LOOKING FOR CONTACT
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  7  TRANSITION TO INITIAL POSITION 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  8  COM TRANSITION TO RIGHT FOOT
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  9  RIGHT FOOT BALANCING
                        80  120   80,  40   40    40   40   40,  40   40    40   40   40,  70   70   70    70   50  50,  80   80  250   200   50  50   % state == 10  YOGA RIGHT FOOT 
                        30   30   30,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50  300    60   50  50   % state == 11  PREPARING FOR SWITCHING 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state == 12  LOOKING FOR CONTACT
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50]; % state == 13  TRANSITION TO INITIAL POSITION

    gain.impedances(1:3,:) = gain.impedances(1:3,:)/5;
    gain.impedances(7:9,:) = gain.impedances(7:9,:)/5;

end              
    
%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.demoOnlyBalancing             = false;
sm.skipYoga                      = false;
sm.demoOnlyRightFoot             = false;
sm.yogaAlsoOnRightFoot           = true;
sm.yogaInLoop                    = false;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     = 5;      % Force threshole above which contact is considered stable
sm.wrench.thresholdContactOff    = 450;    % Force threshole under which contact is considered off
sm.joints                        = struct;
sm.joints.thresholdNotInContact  = 5;      % Degrees
sm.joints.thresholdInContact     = 50;     % Degrees
sm.joints.pauseTimeLastPostureL  = 3;
sm.joints.pauseTimeLastPostureR  = 3;

sm.tBalancingOneFoot             = 1;
sm.stateAt0                      = 1;
sm.DT                            = 1;
sm.waitingTimeAfterYoga          = 0;

sm.jointsSmoothingTimes          = [5;   %% state ==  1  TWO FEET BALANCING
                                    5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    5;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    1;   %% state ==  6  LOOKING FOR CONTACT 
                                    4;   %% state ==  7  TRANSITION INIT POSITION
                                    5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    5;   %% state ==  9  RIGHT FOOT BALANCING 
                                    4;   %% state == 10  YOGA RIGHT FOOT
                                    5;   %% state == 11  PREPARING FOR SWITCHING
                                    5;   %% state == 12  LOOKING FOR CONTACT 
                                    4];  %% state == 13  TRANSITION INIT POSITION

sm.com.states      = [0.00,    0.00,    0.00;       %% state ==  1  TWO FEET BALANCING NOT USED
                      0.00,    0.025,   0.00;       %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.00,    0.00,    0.05;       %% state ==  3  LEFT FOOT BALANCING 
                      0.00,    0.025,   0.25;       %% state ==  4  YOGA LEFT FOOT
                      0.00,    0.000,   0.00;       %% state ==  5  PREPARING FOR SWITCHING
                      0.00,   -0.000,  -0.025;      %% state ==  6  LOOKING FOR CONTACT 
                      0.025,  -0.000,  -0.025;      %% state ==  7  TRANSITION INIT POSITION
                      0.025,  -0.000,   0.00;       %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.00,    0.000,   0.05;       %% state ==  9  RIGHT FOOT BALANCING 
                      0.00,    0.00,    0.25;       %% state == 10  YOGA RIGHT FOOT
                      0.00,    0.00,    0.00;       %% state == 11  PREPARING FOR SWITCHING
                      0.00,    0.050,  -0.025;      %% state == 12  LOOKING FOR CONTACT 
                      0.00,    0.050   -0.025];     %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                  
sm.tBalancing      =  0;

sm.joints.states = [[ 0.0000, 0.0175,-0.0175 ...                           %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                      0.2275, 0.1100,-0.2275,-0.1400, 0.1050 ...           %
                      0.2275,-0.1100, 0.2275,-0.1400,-0.1050 ...           %
                     -0.2450, 0.0000, 0.0000, 0.2975, 0.0000, 0.0000 ...   %
                     -0.2450, 0.0000, 0.0000, 0.2975, 0.0000, 0.0000];     %
                    [-0.0350,-0.2000, 0.0400, ...                          %% state == 2  COM TRANSITION TO LEFT 
                     -0.1875, 0.1000, 0.2500, 0.8700, 0.0000 ...           %
                     -0.1100, 1.4000, 0.0125, 0.8700, 0.0000 ...           %
                     -0.0025,-0.1100,-0.0000, 0.0000, 0.0150, 0.1500, ...  %  
                      0.0000, 0.1000,-0.0025,-0.0050, 0.0075,-0.1150];     %  
                    [ 0.0875,-0.2000, 0.0150, ...                          %% state == 3  LEFT FOOT BALANCING
                      0.0750, 0.1000, 0.3050, 0.7925, 0.0000 ...           %    
                      0.0775, 1.4000, 0.0150, 0.6200, 0.0000 ...           %
                     -0.0015,-0.1100,-0.0000, 0.0000, 0.0150, 0.1650, ...  %  
                      0.0000, 0.0800,-0.0025,-0.0050, 0.0075,-0.1150];     % 
                    [ 0.0875, 0.0275, 0.0150, ...                          %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED 
                      0.1250, 0.8000, 0.3050, 0.7925, 0.0000 ...           %
                      0.0550, 0.7000, 0.0250, 0.6225, 0.0000 ...           %
                      0.0525,-0.2575, 0.0025,-0.2100,-0.0950, 0.1950, ...  %
                      0.0125, 0.4375, 0.0100,-0.1575,-0.0725,-0.2925];     %
                    [-0.0350, 0.0775, 0.0425, ...                          %% state == 5  PREPARING FOR SWITCHING
                     -0.1500, 0.8575, 0.2425, 0.8700, 0.0000 ...           %
                     -0.1500, 0.8575, 0.2425, 0.8700, 0.0000 ...           %
                     -0.0025,-0.1900,-0.0000,-0.0025, 0.0150, 0.1625, ...  %  
                      0.0005, 0.1925,-0.0014,-0.0031  0.0075 -0.1150];     %                                  %
                    [-0.0875, 0.0250, 0.0150, ...                          %% state == 6  LOOKING FOR CONTACT
                      0.1250, 0.9000, 0.3050, 0.7950, 0.0000 ...           %
                      0.0575, 0.9000, 0.3350, 0.6225, 0.0000 ...           %
                      0.0100,-0.0750,-0.0000,-0.0325, 0.0250, 0.1375, ...  %
                     -0.0025, 0.0300, 0.0100,-0.0000, 0.0025,-0.0275];     %   
                      zeros(1,ROBOT_DOF);                                  %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                    [ 0.0875, 0.2000, 0.0150, ...                          %% state == 8  COM TRANSITION TO RIGHT FOOT
                      0.1250, 1.4150, 0.3050, 0.7925, 0.0000 ...           %
                      0.0575, 0.2575, 0.3350, 0.6225, 0.0000 ...           %
                      0.0100,-0.0750,-0.0000,-0.0120, 0.0250, 0.1375, ...  %
                     -0.0025, 0.0300, 0.0100,-0.0000, 0.0025,-0.0275];     % 
                    [ 0.0864, 0.2000, 0.0150, ...                          %% state == 9  RIGHT FOOT BALANCING
                      0.1250, 0.8125, 0.3050, 0.7925, 0.0000 ...           %    
                      0.0575, 0.0800, 0.3350, 0.6225, 0.0000 ...           %
                      0.0005, 0.0800,-0.0025,-0.0050, 0.0075,-0.1150, ...  %  
                     -0.0025,-0.1100,-0.0000, 0.0000, 0.0150, 0.1650];     %  
                      zeros(1,ROBOT_DOF);                                  %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                    [-0.0350, 0.0775, 0.0450, ...                          %% state == 11  PREPARING FOR SWITCHING
                     -0.1500, 0.8600, 0.2425, 0.8700, 0.0000 ...           %
                     -0.1500, 0.8600, 0.2425, 0.8700, 0.0000 ...           %
                      0.0005, 0.0800,-0.0025,-0.0050, 0.0075,-0.1150, ...  %  
                     -0.0025,-0.1100,-0.0000, 0.0005, 0.0150, 0.1625];     %                                  %
                    [ 0.0875, 0.0250, 0.0150, ...                          %% state == 12  LOOKING FOR CONTACT
                      0.1250, 0.8125, 0.3050, 0.7925, 0.0000 ...           %
                      0.0550, 0.6800, 0.3350, 0.6225, 0.0000 ...           %
                     -0.0025, 0.0225, 0.0100,-0.0025, 0.0025,-0.0275, ...  %
                      0.0105,-0.0750,-0.0000,-0.0125, 0.0250, 0.1375];     %   
                      zeros(1,ROBOT_DOF)];                                 %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED                     

 
q1 =        [-0.0800,-0.1000, 0.4500, ...
             -1.2300, 0.7000, 1.2000, 0.9950, 0.0000, ... 
             -1.0700, 1.2000, 0.5500, 1.0950, 0.0000, ...
              0.2100, 0.2950, 0.0005,-0.1750,-0.1045, 0.0700, ...
              0.3485, 0.4000,-0.0005,-0.3650,-0.0550,-0.0875];

q2 =        [-0.0800,-0.1000, 0.4500, ...
             -1.2300, 0.7000, 1.2000, 0.9950, 0.0000, ...
             -1.0700, 1.2000, 0.5500, 1.0950, 0.0000, ...
              0.2090, 0.2950, 0.0005,-0.1750,-0.1045, 0.0700, ... 
              0.3715, 0.9600, 1.3250,-1.6595, 0.6375,-0.0615];
          
q3 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3575, 0.2475, 0.3050, 0.0000, ...
             -0.3180, 1.3575, 0.7375, 0.3050, 0.0000, ...
              0.2090, 0.2950, 0.0005,-0.1750,-0.1045, 0.0700, ...
              0.3715, 0.9600, 1.3255,-1.6595, 0.6375,-0.0615];
          
q4 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.3150, 1.3585, 0.7375, 0.3050, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1045, 0.0700,...
              0.3515, 1.3100, 1.3255,-0.0200, 0.6375,-0.0615];
          
q5 =        [-0.0800,-0.1275, 0.4500, ...
             -1.1625, 0.6675, 0.4965, 0.9945, 0.0000, ...
             -1.0725, 1.2900, 0.2500, 1.0950, 0.0000, ...
              0.2090, 0.3475, 0.0005,-0.1750,-0.1050, 0.0700,...
              0.3525, 1.3100, 1.3250,-0.0200, 0.6370,-0.0600];
          
q6 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.4200, 1.4585, 0.7375, 0.3025, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1075, 0.0700,...
              0.3525, 1.3100, 1.3250,-0.0200, 0.6375,-0.0600];
          
q7 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.4200, 1.4585, 0.7375, 0.3025, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1075, 0.0700,...
              0.3525, 1.3100, 1.3250,-1.6200, 0.6375,-0.0600];
          
q8 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.4200, 1.4585, 0.7375, 0.3025, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1075, 0.0700,...
              0.3525, 1.3100, 1.3250,-0.0200, 0.6375,-0.0600];

%% Symmetry for right foot Yoga          
sm.joints.pointsL = [0,                            q1;
                     1*sm.jointsSmoothingTimes(10),q2;
                     2*sm.jointsSmoothingTimes(10),q3;
                     3*sm.jointsSmoothingTimes(10),q4;
                     4*sm.jointsSmoothingTimes(10),q5;
                     5*sm.jointsSmoothingTimes(10),q6;
                     6*sm.jointsSmoothingTimes(10),q7;
                     7*sm.jointsSmoothingTimes(10),q8];
                 
sm.joints.pointsR = sm.joints.pointsL;
					 
for i = 1:size(sm.joints.pointsR,1)				
	sm.joints.pointsR(i,2:4)           = [sm.joints.pointsR(i,2) -sm.joints.pointsR(i,3) -sm.joints.pointsR(i,4)];
	
	rightArm                           =  sm.joints.pointsR(i,end-16:end-12);
	sm.joints.pointsR(i,end-16:end-12) =  sm.joints.pointsR(i,end-21:end-17);
	sm.joints.pointsR(i,end-21:end-17) =  rightArm;
	
	rightLeg                           =  sm.joints.pointsR(i,end-5:end);
	sm.joints.pointsR(i,end-5:end)     =  sm.joints.pointsR(i,end-11:end-6);
	sm.joints.pointsR(i,end-11:end-6)  =  rightLeg;
end	 

%% Remapping of the references in order to fit the Bigman configuration and the iCub configuration
addpath('../../../../utilityMatlabFunctions/')
numOfStates     = size(sm.joints.states,1);
ndof            = size(sm.joints.states,2);
numOfStatesYoga = 8;

for kk = 1:numOfStates
    
    sm.joints.states(kk,:) = from_iCub_To_Bigman_JointRemapper(sm.joints.states(kk,:),ndof);
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
