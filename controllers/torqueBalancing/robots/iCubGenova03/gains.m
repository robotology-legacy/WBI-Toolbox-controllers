ROBOT_DOF = 23;

directionOfOscillation            = [0;0;0];
referenceParams                   = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

noOscillationTime        = 0; % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                               % that the robot waits before starting the left-and-righ


maxTorque = 24;

smoothingTimeJacobians            = 0.5;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
qTildeMax              = 20*pi/180;

%%
%           PARAMETERS FOR TWO FEET ONE GROUND


if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gainsPCOM                 = diag([ 50   50  50]);
    gainsICOM                 = diag([  0    0   0]);
    gainsDCOM                 = diag([  0    0   0]);

    minCoMx_y                 = [-0.1   -0.25 ];  
    maxCoMx_y                 = [ 0.1    0.05 ];
    satGainsPCOM              = 300;
    increasingRatesGainsPCOM  = [ 0     0    ];

    gainMomentum           = 1 ;

    % Impadances acting in the null space of the desired contact forces 


    % 
    impTorso            = [   40    40   40
                               0     0    0]; 
    impArms             = [13   13   13  15   
                            0    0    0   0   ];
                        
    impLeftLeg          = [ 35   10   40     1200    70  2
                             0    0   0        0      0   0]; 

    impRightLeg         = [ 35   10   40      950    70  2
                             0    0   0        0      0   0]; 
                         
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];                        
                         
                         
    if (DEMO_LEFT_AND_RIGHT == 1)
        directionOfOscillation = [0;1;0];
        referenceParams        = [0.04 0.5];  %referenceParams(1) = amplitude of ascillations in meters
    end
    
end

%%
%           PARAMETERS FOR ONLY ONE FOOT ONE GROUND


if (sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    %%
    gainsPCOM                 = diag([120  140 120]);
    gainsICOM                 = diag([  0    0   0]);
    gainsDCOM                 = diag([  0    0   0]);

    minCoMx_y                 = [-0.1   -0.25 ];  
    maxCoMx_y                 = [ 0.1    0.05 ];
    satGainsPCOM              = 300;
    increasingRatesGainsPCOM  = [ 0     0    ];

    gainMomentum              = 1 ;

    % Impadances acting in the null space of the desired contact forces 

    
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    impTorso            = [  20    20   20
                              0     0    0]; 

    impArms             = [ 13  13   13   5  
                            0    0    0   0              ];

    impLeftLeg          = [ 70   70 650     300      0   0
                             0    0   0       0      0   0]; 

    impRightLeg         = [ 20   20  20      10      0    0
                             0    0   0       0      0   0];

%%    
end

satIntegral         = 0;
integralGains       = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
dampings            = zeros(1,ROBOT_DOF);
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
impedencesSat       = [80   25    1400];

if (size(impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end


%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

footSize                     = [ -0.1 0.1   ;    % xMin, xMax
                                 -0.1 0.1  ];   % yMin, yMax    

fZmin                        = 10;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 0.01;
reg.HessianQP   = 1e-7;