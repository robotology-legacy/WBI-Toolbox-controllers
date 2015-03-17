ROBOT_DOF = 23;

transferCoMyDes                   = -0.04;
transferCoMxDes                   =  0.03;
transferTime                      = 30;
tBalancing                        = 15;
forceThreshold                    = 100;
CoMErrorThreshold                 = 0.1;
smoothingTimeJacobians            = 3;
trajectoryTimePOstural            = 5;
NUMBER_OF_STATES                  = 4;


amplitudesOscillationsOnOneFoot   = zeros(1,ROBOT_DOF);
frequenciesOscillationsOnOneFoot  = zeros(1,ROBOT_DOF);
directionOfOscillation            = [0;1;0];
referenceParams                   = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);


gainsPCOM                 = diag([ 50   50  50]);
gainsICOM                 = diag([  0    0   0]);
gainsDCOM                 = 2*sqrt(gainsPCOM);

minCoMx_y                 = [-0.1   -0.25 ];  
maxCoMx_y                 = [ 0.1    0.05 ];
satGainsPCOM              = 300;
increasingRatesGainsPCOM  = [ 0     0    ];

gainMomentum           = 1 ;

% Impadances acting in the null space of the desired contact forces 


% 
dampTorso            = [1    1   1]*0; 

dampArms             = [1    1   1   1]*0;

dampLeftLeg          = [0.0    0.0   0   0   0   0]; 

dampRightLeg         = [0.0    0.0   0   0   0   0]; 


% 
impTorso            = [   50    50   50
                           0     0    0]; 
impArms             = [8    8    8   12   
                        0   0    0    0];

impLeftLeg          = [ 35    0    30     350    50   0
                         0    0     0       0      0   0]; 

impRightLeg         = [35    0    30      350    50   0
                        0    0     0        0      0   0]; 
                                                  
    
   
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
dampings            = [dampTorso,dampArms,dampArms,dampLeftLeg,dampRightLeg];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
impedencesSat       = [80   100    400];

if (size(impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end