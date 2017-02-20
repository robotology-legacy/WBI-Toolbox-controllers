%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'CHAIR')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally    
  
     %% State parameters
     sm.stateAt0             = 1;
     sm.tBalancing           = 1;
     sm.jointsSmoothingTimes = [1;0.5;0;1];
     
     % smoothing time for time varying impedances
     gain.SmoothingTimeGainScheduling              = 2;  
     
     reg.pinvTol     = 1e-5;
     reg.pinvDamp    = 1; 
     reg.pinvDampVb  = 1e-7;
     reg.HessianQP   = 1e-7;
     reg.impedances  = 0.1;
     reg.dampings    = 0;
     
     %% Contact constraints                                  
     phys.legSize              =  [-0.025  0.025 ;        % xMin, xMax
                                   -0.025  0.025];        % yMin, yMax 
     gain.legSize              =  [-0.025  0.025 ;        % xMin, xMax
                                   -0.025  0.025];        % yMin, yMax 
                                       
     addpath('../../../../utilityMatlabFunctions/')
     [ConstraintsMatrixLegs,bVectorConstraintsLegs] = constraints...
     (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.legSize,fZmin);

     gain.PCOM     =    [50   50  50;    % state ==  1  LEGS BALANCING
                         70   50  50;    % state ==  2  COM TRANSITION
                         30   60  60;    % state ==  3  LOOKING FOR CONTACT
                         50   50  50];   % state ==  4  TWO FEET BALANCING

     gain.ICOM     = gain.PCOM*0;
     gain.DCOM     = 2*sqrt(gain.PCOM)*0;

     gain.PAngularMomentum  = 1 ;
     gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);


                         %   TORSO %%      LEFT ARM    %%      RIGHT ARM   %%         LEFT LEG           %%         RIGHT LEG          %% 
     gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  1  LEGS BALANCING
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  2  COM TRANSITION
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  3  LOOKING FOR CONTACT
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50];  % state ==  4  TWO FEET BALANCING

     gain.impedances(3,:) = gain.impedances(3,:)./2;      
                     
end

                        %%Hip pitch %%Hip roll %%Knee %%Ankle pitch %%Shoulder pitch %%Shoulder roll %%Shoulder yaw %%Elbow %%Torso pitch 
                        
sm.joints.statesChair = [1.5402   0.1594    -1.7365   -0.2814    -1.6455   0.1920   0.5862   0.2473   0.1928;   % state ==  2  COM TRANSITION
                         1.1097   0.0122    -0.8365   -0.0714    -1.4615   0.1920   0.1545   0.2018   0.0611;   % state ==  3  LOOKING FOR CONTACT
                         0.2094   0.1047    -0.1745   -0.0349    -1.6455   0.1920   0.5862   0.2473   0.0];     % state ==  4  TWO FEET BALANCING 
                      
% sm.CoM.statesChair    =  -0.002   0.088  0.17;     % state ==  2  COM TRANSITION

sm.joints.ankleCorrection = -0.1745;

sm.CoM.deltaStatesChair = [-0.0867   0.0  0.0;       % state ==  2  COM TRANSITION
                            0.0      0.0  0.0;      % state ==  3  LOOKING FOR CONTACT
                           -0.0      0.0  0.29];     % state ==  4  TWO FEET BALANCING 
                     
sm.LwrenchTreshold    = [57;    % state ==  2  COM TRANSITION
                         140];  % state ==  3  LOOKING FOR CONTACT
sm.RwrenchTreshold    = [57;    % state ==  2  COM TRANSITION
                         140];  % state ==  3  LOOKING FOR CONTACT
       
   
