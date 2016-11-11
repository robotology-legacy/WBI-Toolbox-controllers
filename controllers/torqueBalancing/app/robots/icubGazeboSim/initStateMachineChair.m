%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'CHAIR')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally    
  
     %% State parameters
     sm.stateAt0             = 1;
     sm.tBalancing           = 1;
     sm.jointsSmoothingTimes = [1;3;1;3];
     
     reg.pinvTol     = 1e-5;
     reg.pinvDamp    = 0.5; 
     reg.pinvDampVb  = 1e-2;
     reg.HessianQP   = 1e-2;
     reg.impedances  = 0.1;
     reg.dampings    = 0;
     
     %% Contact constraints                                  
     phys.legSize              = [ -0.025  0.025 ;        % xMin, xMax
                                   -0.005  0.005];        % yMin, yMax 
     gain.legSize              = [ -0.025  0.025 ;        % xMin, xMax
                                   -0.005  0.005];        % yMin, yMax 
                                       
     addpath('../../../../utilityMatlabFunctions/')
     [ConstraintsMatrixLegs,bVectorConstraintsLegs] = constraints...
     (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.legSize,fZmin);

     gain.PCOM     =    [50   50  50;    % state ==  1  LEGS BALANCING
                         50   50  50;    % state ==  2  COM TRANSITION
                         30   30  30;    % state ==  3  LOOKING FOR CONTACT
                         50   50  50];   % state ==  4  TWO FEET BALANCING

     gain.ICOM     = gain.PCOM*0;
     gain.DCOM     = 2*sqrt(gain.PCOM);

     gain.PAngularMomentum  = 10 ;
     gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);


                         %   TORSO %%      LEFT ARM    %%      RIGHT ARM   %%         LEFT LEG           %%         RIGHT LEG          %% 
     gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  1  LEGS BALANCING
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  2  COM TRANSITION
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  3  LOOKING FOR CONTACT
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50];  % state ==  4  TWO FEET BALANCING

                     
end
                        % Hip pitch % Hip roll % Knee % Ankle pitch % Shoulder pitch % Shoulder roll % Shoulder yaw % Elbow % Torso pitch 
sm.joints.statesChair = [1.5402   0.0594   -1.5365   0.0856   -1.6455   0.1920   0.5862   0.2473   0.1928;
                         1.1097   0.0122   -1.4171   0.1089   -1.4615   0.1920   0.1545   0.2018   0.0611];
                        %1.5402   0.0594   -1.4365  -0.0756   -1.6455   0.1920   0.5862   0.2473   0.1928];
                     
sm.CoM.statesChair    = [0.0869 -0.0861  0.1616;
                         0.1769 -0.0861  0.2116];
                       % 0.0569 -0.0661  0.1716];
                     
sm.LwrenchTreshold    = 100;
sm.RwrenchTreshold    = 100;
       
   
