%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%% WALKMAN SPECIFIC CONFIGURATION SETUP %%%%%%%%%%%%%%%% %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CONFIG.ON_WALKMAN_PILOT_PC = false;
CONFIG.ON_REAL_WALKMAN     = false;

% dynamic calibration parameters
USE_h_ONLY         = true;
USE_JOINT_VELOCITY = 1;
tSwitch            = [10 20 30 40 50 60 70 80 90 100];
tEnd               = tSwitch(end) + 10;

% Simulation time in seconds
CONFIG.SIMULATION_TIME = inf;

% Available movesets: 1 = air_1; 2 = air_2, 3 = yoga
CONFIG.moveset = 3;

if CONFIG.ON_WALKMAN_PILOT_PC

    setenv('CODYCO_SUPERBUILD_ROOT','/home/lucamuratore/src/codyco-superbuild');
    current_path = getenv('PATH');
    setenv('PATH',fullfile(current_path, ':/home/lucamuratore/src/codyco-superbuild/build/install/bin'));
    current_ld_library_path = getenv('LD_LIBRARY_PATH');
    setenv('LD_LIBRARY_PATH',fullfile(current_ld_library_path, ':home/lucamuratore/src/codyco-superbuild/build/install/lib'));
    setenv('YARP_DATA_DIRS','/home/lucamuratore/src/codyco-superbuild/build/install/share/yarp:/home/lucamuratore/src/codyco-superbuild/build/install/share/iCub:/home/lucamuratore/src/codyco-superbuild/build/install/share/codyco');
end

% calibration delta for legs joints
newOffsets = [0.12753
              2.554397
              2.024109
             -2.243889
              1.644721
             -0.39407
             -0.642021
             -0.183969
              0.233145
             -1.176114
              0.866804
              0.247708]; % [deg]

% calibration delta (real - desired) 
calibDelta = newOffsets*pi/180;
PORTS.IMU_CALIB = '/bigman/imu_link/inertial';

if CONFIG.ON_REAL_WALKMAN == 0

    % ONLY FOR SIMULATION
    calibDelta = 0.*calibDelta;
    PORTS.IMU_CALIB = '/bigman/inertial';
end

if CONFIG.moveset == 3
   
    tSwitch  = [10,20,30,40];
    tEnd     = tSwitch(end) + 10;    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ROBOT_DOF              = 25;
CONFIG.ON_GAZEBO       = true;
PORTS.IMU              = '/bigman/inertial';
PORTS.WBD_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBD_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
FRAMES.BASE            = 'Waist';
FRAMES.IMU             = 'imu_link';

CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

CONFIG.SMOOTH_DES_COM              = 0;    % If equal to one, the desired streamed values 
                                           % of the center of mass are smoothed internally 

CONFIG.SMOOTH_DES_Q                = 0;    % If equal to one, the desired streamed values 
                                           % of the postural tasks are smoothed internally 

WBT_wbiList = '(WaistSag,WaistLat,WaistYaw,LShSag,LShLat,LShYaw,LElbj,LForearmPlate,RShSag,RShLat,RShYaw,RElbj,RForearmPlate,LHipSag,LHipLat,LHipYaw,LKneeSag,LAnkSag,LAnkLat,RHipSag,RHipLat,RHipYaw,RKneeSag,RAnkSag,RAnkLat)';

dump.left_wrench_port  = '/bigman/left_leg_ft/analog:o/forceTorque';
dump.right_wrench_port = '/bigman/right_leg_ft/analog:o/forceTorque';

references.smoothingTimeMinJerkComDesQDes    = 3.0;
sat.torque                                   = 500;
CONFIG.smoothingTimeTranDynamics             = 0.05;

ROBOT_DOF_FOR_SIMULINK  = eye(ROBOT_DOF);
gain.qTildeMax          = 20*pi/180;
postures                = 0;  

gain.SmoothingTimeImp            = 1; 
gain.SmoothingTimeGainScheduling = 0.02;

%% PARAMETERS FOR TWO FEET ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gain.PCOM                 = diag([50  50  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 0*sqrt(gain.PCOM);

    gain.PAngularMomentum     = 10 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 

    impTorso            = [10   10   20
                            0    0    0]; 
                        
    impArms             = [10   10    10    10  10 
                            0    0     0     0   0 ];
                        
    impLeftLeg          = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 

    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 
    
                         
    intTorso            = [0   0    0]; 
    
    intArms             = [0   0    0    0  0];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0     0  0    0  0];   
                                               
end

%% PARAMETERS FOR ONLY ONE FOOT ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 
    
    intTorso            = [0   0    0];
    
    intArms             = [0   0    0    0  0];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp;
                        
    impArms             = [15   15    15   10  10
                            0    0     0    0   0 ]*scalingImp;
                        
    impLeftLeg          = [ 30   30   30   120   10  10
                             0    0    0     0    0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]*scalingImp; 
                            
end

%%  Config the gains
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

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot
phys.footSize                = [ -0.16  0.16   ;   % xMin, xMax
                                 -0.075 0.075 ];   % yMin, yMax    
                             
gain.footSize                = [ -0.16  0.16   ;   % xMin, xMax
                                 -0.075 0.075 ];   % yMin, yMax    

fZmin                        = 10;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 0.01;
reg.pinvDampVb  = 1e-7;
reg.HessianQP   = 1e-5;
reg.impedances  = 0.1;
reg.dampings    = 0;
