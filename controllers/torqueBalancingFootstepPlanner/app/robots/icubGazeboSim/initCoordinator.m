%% Configuration parameters for COODINATOR demo
CONFIG.DEMO_MOVEMENTS = false; % Either true or false
CONFIG.ON_GAZEBO      = true;
ROBOT_DOF             = 23;
PORTS.IMU             = '/icubSim/inertial';

CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];
CONFIG.SMOOTH_DES_COM             = 0;    % If equal to one, the desired streamed values 
                                          % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q               = 0;    % If equal to one, the desired streamed values 
                                          % of the postural tasks are smoothed internally 

WBT_wbiList   = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';
WBT_robotName = 'icubSim';

dump.left_wrench_port  = '/icubSim/left_foot/analog:o';
dump.right_wrench_port = '/icubSim/right_foot/analog:o';

references.smoothingTimeMinJerkComDesQDes = 3.0;
CONFIG.smoothingTimeTranDynamics          = 0.05;

sat.torque = 60;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
gain.qTildeMax         = 20*pi/180;
postures               = 0;  

gain.SmoothingTimeImp            = 1; 
gain.SmoothingTimeGainScheduling = 0.02;

%% PARAMETERS FOR TWO FEET ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
   
    gain.PCOM                 = diag([50    50  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 0*sqrt(gain.PCOM);

    gain.PAngularMomentum     = 10 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 

    impTorso            = [10   10   20
                            0    0    0]; 
    impArms             = [10   10    10    8   
                            0    0     0    0 ];                       
    impLeftLeg          = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 
    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 
                          
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0    0];                 
    intLeftLeg          = [0   0    0    0   0  0];
    intRightLeg         = [0   0    0    0   0  0];                                             
end

%% PARAMETERS FOR ONLY ONE FOOT ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)

    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 1 ;

    % Impedances acting in the null space of the desired contact forces 
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0   0];                
    intLeftLeg          = [0   0    0   0   0  0]; 
    intRightLeg         = [0   0    0   0   0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp; 
    impArms             = [15   15    15    8 
                            0    0     0    0]*scalingImp;                   
    impLeftLeg          = [ 30   30   30   120     10  10
                             0    0    0     0      0   0]*scalingImp; 
    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]*scalingImp; 
                             
end

sat.integral              = 0;
gain.integral             = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances           = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = 0*sqrt(gain.impedances);
gain.increasingRatesImp   = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;
torsionalFrictionCoefficient = 2/150;

% physical size of foot
phys.footSize                = [ -0.07 0.07  ;   % xMin, xMax
                                 -0.03 0.03 ];   % yMin, yMax    
                             
gain.footSize                = [ -0.07 0.07  ;   % xMin, xMax
                                 -0.03 0.03 ];   % yMin, yMax    

fZmin                        = 10;

%% LEFT AND RIGHT DEMO 
references.com.points             = 0;
references.com.noOscillationTime  = 0; % that the robot waits before starting the left-and-right
references.joints.points          = 0;

if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2 )
    if (CONFIG.DEMO_MOVEMENTS)
        references.com.directionOfOscillation  = [0;1;0];
        references.com.amplitudeOfOscillation  = 0.02;
        references.com.frequencyOfOscillation  = 0.2;
    else
        references.com.directionOfOscillation  = [0;0;0];
        references.com.amplitudeOfOscillation  = 0.0;  % amplitude of oscillations in meters 
        references.com.frequencyOfOscillation  = 0.0;  % frequency of oscillations in hertz
    end
else

    q1 = [-0.0790    0.2279    0.4519 ...   
          -1.1621    0.6663    0.4919    0.9947  ...  
          -1.0717    1.2904   -0.2447    1.0948  ...   
           0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
           0.3484    0.4008   -0.0004   -0.3672   -0.0530   -0.0875];

    q2 = [-0.0790    0.2279    0.4519 ...
          -1.1621    0.6663    0.4965    0.9947  ...
          -1.0717    1.2904   -0.2493    1.0948  ...
           0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
           0.3714    0.1906    1.3253   -0.9794    0.6374   -0.0614 ];

    q3 = [-0.0852   -0.4273    0.0821 ...
           0.1391    1.4585    0.2464    0.3042   ... 
          -0.4181    1.6800    0.7373    0.3031  ... 
           0.3850    0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
           0.3514    1.3107    1.3253   -0.0189    0.6374   -0.0614 ];

    q4 = [-0.0179  0.3218    0.0076 ...
          -0.6494  1.6296    0.0013    0.8756  ...
          -0.6524  0.8722    0.0012    0.6122  ...
           0.3850  0.4889   -0.0001   -0.2958   -0.0990    0.0249 ...
           0.2091  0.2940    0.0001   -0.1738   -0.1062    0.0781 ];

    references.joints.points = [ references.com.noOscillationTime,  q1;
                                 references.com.noOscillationTime+  references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+4*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+5*references.joints.smoothingTime, q4
                                 references.com.noOscillationTime+6*references.joints.smoothingTime, q1;
                                 references.com.noOscillationTime+7*references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+8*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+9*references.joints.smoothingTime, q4
                                 references.com.noOscillationTime+10*references.joints.smoothingTime, q1;
                                 references.com.noOscillationTime+11*references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+12*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+13*references.joints.smoothingTime, q4
                                 references.com.noOscillationTime+14*references.joints.smoothingTime, q1;
                                 references.com.noOscillationTime+15*references.joints.smoothingTime, q2;
                                 references.com.noOscillationTime+16*references.joints.smoothingTime, q3;
                                 references.com.noOscillationTime+17*references.joints.smoothingTime, q4];
    clear q1 q2 q3 q4;
end

%% Regularization terms
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 0.01;
reg.pinvDampVb  = 1e-7;
reg.HessianQP   = 1e-5;
reg.impedances  = 0.1;
reg.dampings    = 0;
