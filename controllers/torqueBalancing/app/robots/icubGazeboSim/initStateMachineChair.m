%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'CHAIR')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally    
  
     %% State parameters
     sm.stateAt0             = 1;
     sm.tBalancing           = inf;
     sm.jointsSmoothingTimes = [5;5;4;3];
     
     reg.pinvTol     = 1e-5;
     reg.pinvDamp    = 1; 
     reg.pinvDampVb  = 1e-2;
     reg.HessianQP   = 1e-2;
     reg.impedances  = 0.1;
     reg.dampings    = 0;
     
     %% Contact constraints                                  
     gain.legSize              = [ -0.025  0.025 ;        % xMin, xMax
                                   -0.005  0.005];        % yMin, yMax 
                                       
     addpath('../../../../utilityMatlabFunctions/')
     [ConstraintsMatrixLegs,bVectorConstraintsLegs] = constraints...
     (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.legSize,fZmin);

     gain.PCOM     =    [50   50  50;    % state ==  1  LEGS BALANCING
                         50   50  50;    % state ==  2  COM TRANSITION
                         50   50  50;    % state ==  3  LOOKING FOR CONTACT
                         50   50  50];   % state ==  4  TWO FEET BALANCING

     gain.ICOM     = gain.PCOM*0;
     gain.DCOM     = 2*sqrt(gain.PCOM);

     gain.PAngularMomentum  = 10 ;
     gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);


     %                   %   TORSO %%      LEFT ARM    %%      RIGHT ARM   %%         LEFT LEG           %%         RIGHT LEG          %% 
     gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  1  LEGS BALANCING
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  2  COM TRANSITION
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  3  LOOKING FOR CONTACT
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50];  % state ==  4  TWO FEET BALANCING
    
end              
