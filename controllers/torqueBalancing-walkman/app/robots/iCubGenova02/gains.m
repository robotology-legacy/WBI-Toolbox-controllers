ROBOT_DOF = 23;

WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';

gain.SmoothingTimeGainScheduling              = 2;  


CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

CONFIG.SMOOTH_DES_COM      = 0;    % If equal to one, the desired streamed values 
                            % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q        = 0;    % If equal to one, the desired streamed values 
                            % of the postural tasks are smoothed internally 
                            
PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';

references.smoothingTimeMinJerkComDesQDes    = 3.0;

sat.torque = 34;

CONFIG.smoothingTimeTranDynamics    = 0.05;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
gain.qTildeMax         = 20*pi/180;
postures = 0;  

gain.SmoothingTimeImp  = 1;  

%%
%           PARAMETERS FOR TWO FEET ONE GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gain.PCOM                 = diag([50   100  5]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 0*2*sqrt(gain.PCOM)/20;

    gain.PAngularMomentum     = 5 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impadances acting in the null space of the desired contact forces 

    impTorso            = [10   10   20
                            0    0    0]; 
    impArms             = [10   10    10    8   
                            0    0     0    0   ];
                        
    impLeftLeg          = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 
    
                         
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0     0  0    0  0];   
    
                                           
end

% PARAMETERS FOR ONLY ONE FOOT ONE GROUND

if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    %%
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 1 ;

    % Impadances acting in the null space of the desired contact forces 

    
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp; 
    impArms             = [15   15    15    8   
                            0    0     0    0   ]*scalingImp;
                        
    impLeftLeg          = [ 30   30   30   120     10  10
                             0    0    0     0      0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]*scalingImp; 
                            
%%    
end

sat.integral              = 0;
gain.integral            = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings            = zeros(1,ROBOT_DOF);
gain.increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end


%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1/3;  
torsionalFrictionCoefficient = 2/150;
                      
gain.footSize  = [ -0.07  0.12   ;    % xMin, xMax
                       -0.04 0.04 ];   % yMin, yMax   

% gain.footSize                = [ -0.065 0.13   ;    % xMin, xMax
%                                  -0.04 0.04  ];   % yMin, yMax
fZmin                        = 1;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 2;
reg.pinvDampVb  = 1e-7;
reg.HessianQP   = 1e-7;
reg.impedances  = 0.1;
reg.dampings    = 0;