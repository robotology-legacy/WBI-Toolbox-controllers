ROBOT_DOF = 23;

CoMDes                            = [   0  ,-0.01,0;
                                      0.055,-0.05,0.523700057608765];
transferTime                      = 1;
tBalancing                        = 1;
forceThreshold                    = 40;
CoMErrorThreshold                 = 0.01;
smoothingTimeJacobians            = 0.5;
trajectoryTimePostural            = transferTime;
NUMBER_OF_STATES                  = 4;


qDesRightFoot   = [ -0.0549299228319975,-0.0103377194156588,0.332187941148811,...
                    -0.625058025521925,0.506720866348244,0.146634215133112,0.882695474335550,...
                    -0.600700133763323,0.514047413318154,0.0774772368758000,0.858491018115584,...
                    0.161644339953937,-0.0968178248798615,-0.000920613231821383,-0.253878278117021,-0.0568670465072878,0.183547263094349,...
                    0.345306679702263,0.107730927565408,0.0164751409611334,-0.325743648526063,-0.0638867223999242,-0.153704050829479;
                    -0.183815775286963,0.0418303637208750,0.315310031898756,...
                    -0.626592380908294,0.551217172552935,0.0330919165418325,0.888832895881025,...
                    -0.593028356831480,0.463413685567989,-0.0744239463746956,0.893781192002063,...
                    0.160109984567568,-0.0845429817889123,-0.00245496861819003,-0.261550055048864,-0.207233874371415,0.0592644767984888,...
                    0.356047167406843,0.107730927565408,0.0195438517338707,-0.258232011525842,-0.0301309038998141,-0.113810810783894];
            
            
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
dampTorso            = [1    1   1]*0; 

dampArms             = [1    1   1   1  ]*0;

dampLeftLeg          = [0.0    0.0   0   0   0   0]; 

dampRightLeg         = [0.0    0.0   0   0   0   0]; 


% 
impTorso            = [   40    40   40
                           0     0    0]; 
impArms             = [13   13   13  15   
                        0    0    0   0   ];

impLeftLeg          = [ 35   10   40     12    70  2
                         0    0   0        0      0   0]; 

impRightLeg         = [ 35   10   40      95     10  2
                         0    0   0        0      0   0]; 

    
   
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
dampings            = [dampTorso,dampArms,dampArms,dampLeftLeg,dampRightLeg];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
impedencesSat       = [80   100    400];

if (size(impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end
