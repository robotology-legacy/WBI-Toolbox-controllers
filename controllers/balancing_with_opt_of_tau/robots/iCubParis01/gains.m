ROBOT_DOF = 23;

amplitudesOscillationsOnOneFoot   = zeros(1,ROBOT_DOF);
frequenciesOscillationsOnOneFoot  = zeros(1,ROBOT_DOF);
directionOfOscillation            = [0;1;0];
referenceParams                   = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
qTildeMax              = 20*pi/180;
% Controller gains for convergence of the desired centroidal momentum. 
% The first three elements are the Proportional, Intagral, and the Derivative
% gains taking place in xComDDStart, i.e. 
%
% xComDDStart = xDDcomDes - Gains(1)*(xcom - xcomDes) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - xDcomDes)  
%
% The fourth element is the gain for the 
% angular part of the centroidal momentum convergence, i.e. 
%
% hwDot = -Gains(4)*hw  


%%
%           PARAMETERS FOR TWO FEET ONE GROUND


if (number_of_feet_on_ground == 2)
    gainsPCOM                 = diag([ 30   50  50]);
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
    impArms             = [15   15   20   12    
                            0    0    0    0   ];
                        
    impLeftLeg          = [ 35   10   40      100    70  2
                             0    0   0         0     0   0]; 

    impRightLeg         = [ 35   10   40      100    70   2
                             0    0   0         0     0   0]; 
    
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];                        
                         
                         
    if (DEMO_LEFT_AND_RIGHT == 1)
        directionOfOscillation = [0;1;0];
        referenceParams        = [0.04 0.5];  %referenceParams(1) = amplitude of ascillations in meters
    end
    
    if (DEMO_MOVING_LEG_AND_ARMS == 1)
        amplTorso            = [ 10  20   30 ]; 
        amplArms             = [ 20  20   20   0];
        amplLeftLeg          = [  0   0    0   0   0   0]; 
        amplRightLeg         = [  0   0    0   0   0   0];

        freqTorso            = [ 0.2  0.1  0.2]; 
        freqArms             = [ 0.2  0.2  0.2  0.2];
        freqLeftLeg          = [ 0.0  0.0  0.0  0.0  0.0  0.0]; 
        freqRightLeg         = [ 0.0  0.0  0.0  0.0  0.0  0.0]; %[ 0.0  0.3  0.0  0.0  0.0  0.0];

        amplitudesOscillationsOnOneFoot   = [amplTorso,amplArms,amplArms,amplLeftLeg,amplRightLeg];
        frequenciesOscillationsOnOneFoot  = [freqTorso,freqArms,freqArms,freqLeftLeg,freqRightLeg];
    end
end

%%
%           PARAMETERS FOR ONLY ONE FOOT ONE GROUND


if (number_of_feet_on_ground == 1)
    %%
    gainsPCOM                 = diag([120  140 120])/3;
    gainsICOM                 = diag([  0    0   0]);
    gainsDCOM                 = diag([  1    1   1]);

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
    
    if (DEMO_MOVING_LEG_AND_ARMS == 0)
        impTorso            = [  20    20   20
                                  0     0    0]; 

        impArms             = [ 13  13   13   5  
                                0    0    0   0              ];

        impLeftLeg          = [ 70   70  65      30      0   0
                                 0    0   0       0      0   0]; 

        impRightLeg         = [ 20   20  20      10      0    0
                                 0    0   0       0      0   0];

    else
        impTorso            = [  70    20   50
                                  0     0    0]; 

        impArms             = [ 13  13   13   10  
                                0    0    0   0              ];

        impLeftLeg          = [ 70   70 650     300      0   0
                                 0    0   0       0      0   0]; 

        impRightLeg         = [ 40   40  20      10     10   10
                                 0    0   0       0      0   0];

        amplTorso            = [ 10  10   10 ]; 
        amplArms             = [  0   0    0   0];
        amplLeftLeg          = [  0   0    0   0   0   0]; 
        amplRightLeg         = [ 25  25   25  15   0   0];

        freqTorso            = [ 0.0  0.0  0.0]; 
        freqArms             = [ 0.0  0.0  0.0  0.0];
        freqLeftLeg          = [ 0.0  0.0  0.0  0.0  0.0  0.0]; 
        freqRightLeg         = [ 0.2  0.2  0.0  0.0  0.0  0.0]; %[ 0.0  0.3  0.0  0.0  0.0  0.0];

        amplitudesOscillationsOnOneFoot   = [amplTorso,amplArms,amplArms,amplLeftLeg,amplRightLeg];
        frequenciesOscillationsOnOneFoot  = [freqTorso,freqArms,freqArms,freqLeftLeg,freqRightLeg];
    end
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