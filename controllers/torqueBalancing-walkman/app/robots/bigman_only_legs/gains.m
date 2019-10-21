ROBOT_DOF              = 12;
CONFIG.ON_GAZEBO       = true;
PORTS.IMU              = '/bigman/inertial';
PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
FRAMES.BASE            = 'Waist';
FRAMES.IMU             = 'imu_link';

CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

CONFIG.SMOOTH_DES_COM              = 0;    % If equal to one, the desired streamed values 
                                           % of the center of mass are smoothed internally 

CONFIG.SMOOTH_DES_Q                = 0;    % If equal to one, the desired streamed values 
                                           % of the postural tasks are smoothed internally 

WBT_wbiList = '(LHipSag,LHipLat,LHipYaw,LKneeSag,LAnkSag,LAnkLat,RHipSag,RHipLat,RHipYaw,RKneeSag,RAnkSag,RAnkLat)';

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
    impLeftLeg          = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 

    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 
                   
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
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
                        
    impLeftLeg          = [ 30   30   30   120   10  10
                             0    0    0     0    0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]*scalingImp; 
                            
end

%%  Config the gains
sat.integral              = 0;
gain.integral             = [intLeftLeg,intRightLeg];
gain.impedances           = [impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = 0*sqrt(gain.impedances);
gain.increasingRatesImp   = [impLeftLeg(2,:),impRightLeg(2,:)];
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
