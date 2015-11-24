%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if USE_SM
    reg.pinvDamp    = 0.1;
    sat.torque = 50;

    references.joints.smoothingTime    = 5;
    references.com.smoothingTime       = references.joints.smoothingTime;
    gain.SmoothingTimeImp              = references.joints.smoothingTime;  

    smoothingTimeTransitionDynamics    = 0.02;


    gain.PCOM              = diag([50    50  50]);
    gain.ICOM              = diag([  0    0   0]);
    gain.DCOM              = 2*sqrt(gain.PCOM)*0;

    gain.PAngularMomentum  = 1 ;


    % state == 1  TWO FEET BALANCING
    % state == 2  COM TRANSITION TO LEFT 
    % state == 3  LEFT FOOT BALANCING 
    % state == 4  YOGA LEFT FOOT  
    % state == 5  PREPARING FOR SWITCHING
    % state == 6  LOOKING FOR CONTACT 
    % state == 7  BALANCING TWO FEET


    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [10   10   20, 10   10    10    8, 10   10    10    8, 30   50   30    60      5   5, 30   30   20    20     10  10  % state == 1  TWO FEET BALANCING
                        10   10   20, 10   10    10    8, 10   10    10    8, 30   50   30    60      5   5, 30   30   20    20     10  10  % state == 2  COM TRANSITION TO LEFT 
                        10   10   20, 10   10    10    8, 10   10    10    8, 30   50   30    60      5   5, 30   30   20    20     10  10  % state == 3  LEFT FOOT BALANCING
                        30   30   30, 10   10    20   10, 10   10    20   10,100   50   30   100     25  25,100   90   20    20     10  10  % state == 4  YOGA LEFT FOOT 
                        30   30   30, 10   10    20   10, 10   10    20   10,220  550  220   200     65 300,200  250   20    20     10  10  % state == 5  PREPARING FOR SWITCHING 
                        30   30   30, 10   10    20   10, 10   10    20   10,220  550  220   200     65 300,100  350   20   200     10 100  % state == 6  LOOKING FOR CONTACT
                        30   50   30, 10   10    20   10, 10   10    20   10, 30   30   20    20     10  10, 30   30   20    20     10  10];% state == 7  BALANCING TWO FEET 

end              
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         
%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.com.threshold          =  0.005;
sm.wrench.threshold       = 25;
sm.joints.thresholdRL     =  3;
sm.joints.thresholdLL     =  3;

sm.stateAt0               =  1;

sm.DT                     =  1;
sm.waitingTimeAfterYoga   = 20;



sm.com.states      = [0.0, -0.14,0.511;   %% CoM reference state 1
                      0.0, -0.04,0.511;   %% CoM reference state 2
                      0.0, -0.14,0.511;   %% CoM reference state 3
                      0.0, -0.14,0.511;   %% CoM reference state 4
                      0.0, -0.14,0.511;   %% CoM reference state 5
                      0.0, -0.09,0.511;   %% CoM reference state 6
                      0.0, -0.09,0.511];  %% CoM reference state 7

sm.tBalancing      = 0;


sm.joints.states = [[0.0864,0.0258,0.0152, ...                          %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928 ...                    %
                     0.0563,0.6789,0.3340,0.6214 ...                    %
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 2  COM TRANSITION TO LEFT 
                     -0.1493,0.8580,0.2437,0.8710 ...                   %
                     -0.1493,0.8580,0.2437,0.8710 ...                   %
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %  
                    [0.0864,0.0258,0.0152, ...                          %% state == 3  LEFT FOOT BALANCING
                     0.1253,0.8135,0.3051,0.7928 ...                    %    
                     0.0563,0.6789,0.3340,0.6214 ...                    %
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %  
                    [0.0864,0.0258,0.0152, ...                          %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928 ...                    %
                     0.0563,0.6789,0.3340,0.6214 ...                    %
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %  
                    [-0.0348,0.0779,0.0429, ...                         %% state == 5  PREPARING FOR SWITCHING
                     -0.1493,0.8580,0.2437,0.8710 ...                   %
                     -0.1493,0.8580,0.2437,0.8710 ...                   %
                      0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ... %  
                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];    %                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 6  LOOKING FOR CONTACT
                     0.1253,0.8135,0.3051,0.7928 ...                    %
                     0.0563,0.6789,0.3340,0.6214 ...                    %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277,...   %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];     %   
                    [0.0864,0.0258,0.0152, ...                          %% state == 7  BALANCING TWO FEET, THIS REFERENCE IS IGNORED 
                     0.1253,0.8135,0.3051,0.7928 ...                    %
                     0.0563,0.6789,0.3340,0.6214 ...                    %
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931]];    %

 
q1 =        [-0.0790,0.2279, 0.4519, ...
             -1.1621,0.6663, 0.4919, 0.9947, ... 
             -1.0717,1.2904,-0.2447, 1.0948, ...
              0.2092,0.2960, 0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3484,0.4008,-0.0004,-0.3672,-0.0530,-0.0875];

q2 =        [-0.0790,0.2279, 0.4519, ...
             -1.1621,0.6663, 0.4965, 0.9947, ...
             -1.0717,1.2904,-0.2493, 1.0948, ...
              0.2092,0.2960, 0.0006,-0.1741,-0.1044,0.0700, ... 
              0.3714,0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
q3 =        [-0.0852,-0.4273,0.0821,...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092,0.2960, 0.0006,-0.1741,-0.1044,0.0700, ...
              0.3714,0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
q4 =        [-0.0852,-0.4273,0.0821,...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q5 =        [-0.0790,0.2279, 0.4519, ...
             -1.1621,0.6663, 0.4965, 0.9947, ...
             -1.0717,1.2904,-0.2493, 1.0948, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q6 =        [-0.0852,-0.4273,0.0821,...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q7 =        [-0.0852,-0.4273,0.0821,...
              0.1391, 1.4585,0.2464, 0.3042, ...
             -0.4181, 1.6800,0.7373, 0.3031, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253, -1.6217, 0.6374,-0.0614];
          
q8 =        [-0.0852,-0.4273,0.0821,...
             -0.2793,1.4312,-0.5585,1.5708, ...
             -0.2793,1.4312,-0.5585,1.5708, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
              
          
          
sm.joints.points = [ 0,                                q1;
                     references.joints.smoothingTime,  q2;
                     2*references.joints.smoothingTime,q3;
                     3*references.joints.smoothingTime,q4
                     4*references.joints.smoothingTime,q5
                     5*references.joints.smoothingTime,q6
                     6*references.joints.smoothingTime,q7
                     7*references.joints.smoothingTime,q8];
					 
for i = 1:size(sm.joints.points,1)				
	sm.joints.points(i,2:4)          = [sm.joints.points(i,2) -sm.joints.points(i,3) -sm.joints.points(i,4)];
	
	rightArm                          =  sm.joints.points(i,end-15:end-12);
	sm.joints.points(i,end-15:end-12) =  sm.joints.points(i,end-19:end-16);
	sm.joints.points(i,end-19:end-16) =  rightArm;
	
	rightLeg                          =  sm.joints.points(i,end-5:end);
	sm.joints.points(i,end-5:end)     =  sm.joints.points(i,end-11:end-6);
	sm.joints.points(i,end-11:end-6)  =  rightLeg;
end	 
                 
% sm.joints.points = [ references.joints.smoothingTime,q9];

clear q1 q2 q3 q4;











sm.joints.smoothingTime    = references.joints.smoothingTime;
sm.com.smoothingTime       = references.com.smoothingTime;
