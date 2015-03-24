ROBOT_DOF = 23;

transferCoMyDes                   = -0.005;
transferCoMxDes                   =  0.03;
transferTime                      = 5.5;
tBalancing                        = 1;
forceThreshold                    = 100;
CoMErrorThreshold                 = 0.1;
smoothingTimeJacobians            = 0.5;
trajectoryTimePOstural            = transferTime;
NUMBER_OF_STATES                  = 4;

qDesRightFoot   = [-0.00889926124093822,0.00500583444802759,-0.00230153307955296,...
                   -0.643470290158349,0.505186510961875,0.0898630658374722,0.888832895881025,...
                   -0.631387241490696,0.518650479477260,0.0406527076029526,0.890712481229326,...
                    0.206140646158627,-0.107558312584442,0.000613742154547258,-0.178694864184958,-0.0599357572800251,0.180478552321611,...
                    0.498742218339127,0.133814969133675,-0.000402768288921669,-0.403995773230863,-0.0684897885590301,-0.176719381625008;
                    -0.0196397489455187,0.0188150329253454,0.0498665500569808,...
                    -0.605111405499133,0.508255221734613,0.135893727428531,0.878092408176444,...
                    -0.625249819945221,0.487963371749887,0.0207060875801602,0.887643770456589,...
                    0.189262736908572,-0.139779775698183,0.00214809754091590,-0.192504062662276,-0.202630808212309,0.0669362537303320,...
                    0.509482706043708,0.150692878383730,-0.000402768288921669,-0.365636888571647,-0.0746272101045047,-0.126085653874843];
            
            
qDesCoM      = [-0.00122748430909501,0.00960890060713352,0.0145763761705021,...
                -0.628126736294663,0.534339263302879,0.140496793587637,0.871954986630970,...
                -0.629852886104327,0.507909991772680,0.0529275506939017,0.870765861206533,...
                 0.190797092294941,-0.152054618789133,0.000613742154547258,-0.190969707275907,-0.0660731788254996,0.160531932298819,...
                 0.168855810269869,0.133814969133675,-0.000402768288921669,-0.153895845252775,-0.0470088131498692,-0.169047604693165];
             
amplitudesOscillationsOnOneFoot   = zeros(1,ROBOT_DOF);
frequenciesOscillationsOnOneFoot  = zeros(1,ROBOT_DOF);
directionOfOscillation            = [0;1;0];
referenceParams                   = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);


gainsPCOM                 = diag([100  100 100])/3;
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

dampArms             = [1    1   1   1  ]*0;

dampLeftLeg          = [0.0    0.0   0   0   0   0]; 

dampRightLeg         = [0.0    0.0   0   0   0   0]; 


% 
impTorso            = [   60    20   10
                           0     0    0]; 
impArms             = [8    8    8  12   
                        0    0    0   0   ];

impLeftLeg          = [160  160    10      350    350  100
                         0    0   0        0      0   0]; 

impRightLeg         = [160  160    10      350    350  100
                        0    0   0       0      0   0]; 
                                                  
    
   
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
dampings            = [dampTorso,dampArms,dampArms,dampLeftLeg,dampRightLeg];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
impedencesSat       = [80   100    400];

if (size(impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end














