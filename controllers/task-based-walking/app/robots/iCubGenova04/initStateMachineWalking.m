% INITSTATEMACHINEWALKING initializes the robot configuration for running
%                         the walking demo. 
%
% USAGE: please note that this function is automatically executed when
%        running the Simulink model.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---

% ROBOT SETUP 
%
% If true, the feet are controlled also when in contact 
Config.controlFeetWhenInContact = false;

% Joint torque saturation
Sat.tau_max = 50; % [Nm]

% Saturation on torque derivative (for QP solver)
Sat.tauDot_max = 1000;

% Weight for the torque and forces minimization task
Sat.weight_tau             = 0.1;
Sat.weight_fRight          = 0.01;
Sat.weight_fLeft           = 0.01;
Sat.weight_tauC            = 0.0;
Sat.weight_rotationalTask  = 0.1;
Sat.weight_Feet            = 0.0;
Config.feetInCostWhenSwing = true;
% Config.rotationalTaskInCost = true;

% Numerical tolerance for assuming a foot on contact and for rotational PID activation
Sat.toll_feetInContact = 0.1;
Sat.toll_useRotPID     = 0.1;

% Damping for the pseudoinverse used for computing the floating base velocity
Sat.pinvDamp_nu_b = 0.1;

%% Parameters for motors reflected inertia

% transmission ratio
Config.Gamma = 0.01*eye(ROBOT_DOF);

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% parameters for coupling matrices                            
t  = 0.625;
r  = 0.022;
R  = 0.04;

% coupling matrices
T_LShoulder = [-1  0  0;
               -1 -t  0;
                0  t -t];

T_RShoulder = [ 1  0  0;
                1  t  0;
                0 -t  t];

T_torso = [0   -0.5     0.5;
           0    0.5     0.5;
           r/R  r/(2*R) r/(2*R)];
       
if Config.INCLUDE_COUPLING
       
    Config.T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));
else          
    Config.T = eye(ROBOT_DOF);
end

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff = 0;

%% Smoothing of reference trajectories and base 

% If true, reference trajectories are smoothed internally
Config.SMOOTH_COM_REF      = false;
Config.SMOOTH_LFOOT_POS    = true;
Config.SMOOTH_RFOOT_POS    = true;
Config.SMOOTH_LFOOT_ORIENT = true; 
Config.SMOOTH_RFOOT_ORIENT = true; 
Config.SMOOTH_ROT_TASK_REF = false;
Config.SMOOTH_JOINT_REF    = true; 

% If true, the base pose and velocity are smoothed internally
Config.SMOOTH_BASE_VEL     = false;
Config.SMOOTH_BASE_POS     = false;
Config.SMOOTH_BASE_ORIENT  = false;

% Smoothing time for tasks and joints references [s]
Config.smoothingTime_CoM    = [0.1;0.1;0.1];
Config.smoothingTime_LFoot  = [0.2;0.2;0.2];
Config.smoothingTime_RFoot  = [0.2;0.2;0.2];
Config.smoothingTime_joints = [0.2;0.2;0.2];

% Gains that will influence the smoothing of reference orientations. The
% lower are, the more smoothed is the trajectory. Only positive or null values.
Config.LFoot_Kp_smoothing        = [1;1;1];
Config.RFoot_Kp_smoothing        = [1;1;1];
Config.rot_task_Kp_smoothing     = [1;1;1];
Config.baseRotation_Kp_smoothing = 10;

% Smoothing time for gain scheduling [s].
Config.smoothingTimeGains        = [0.2;0.2;0.2];

% Minimum value of the vertical force at contact location for the contact
% to be considered as active 
Config.threshold_contact_twoFeet = 20;  % [N]
Config.threshold_contact_on      = 180; % [N]
Config.threshold_contact_off     = 200; % [N]

%% Gains matrices

% CoM position and velocity gains
Gains.Kp_CoM = [60, 70, 70; ...  % state = 1 two feet balancing
                60, 70, 70; ...  % state = 2 left foot balancing
                60, 70, 70];   % state = 3 right foot balancing
                
Gains.Kd_CoM = [2*sqrt(Gains.Kp_CoM(:,1))/7, 2*sqrt(Gains.Kp_CoM(:,1))/7, 2*sqrt(Gains.Kp_CoM(:,1))/10];

% Feet position and velocity gains              
Gains.Kp_LFoot = [75, 75, 100, 500, 500, 500; ...   % state = 1 two feet balancing
                  75, 75, 100, 500, 500, 500; ...   % state = 2 left foot balancing
                  75, 75, 100, 500, 500, 500];      % state = 3 right foot balancing
              
Gains.Kd_LFoot = 2*sqrt(Gains.Kp_LFoot)/50;

Gains.Kp_RFoot = [75, 75, 100, 500, 500, 500; ...   % state = 1 two feet balancing
                  75, 75, 100, 500, 500, 500; ...   % state = 2 left foot balancing
                  75, 75, 100, 500, 500, 500];      % state = 3 right foot balancing

Gains.Kd_RFoot = 2*sqrt(Gains.Kp_RFoot)/50; 

% Root link orientation and angular velocity gains
Gains.Kp_rot_task = [100, 100, 100; ...    % state = 1 two feet balancing
                     100, 100, 100; ...    % state = 2 left foot balancing
                     100, 100, 100]/2;     % state = 3 right foot balancing
                 
Gains.Kd_rot_task =  2*sqrt(Gains.Kp_rot_task)/40; 

% Joint position and velocity gains
    
                    % torso     % left arm   % right arm  % left leg              % right leg                                   
Gains.impedances = [60  60  60, 50 50 50 50, 50 50 50 50, 80 200 100 120 200 200, 80 200 100 120 200 200;  ...     % state = 1 two feet balancing          
                    60  60  60, 50 50 50 50, 50 50 50 50, 80 200 100 120 200 200, 80 200 100 120 200 200;  ...     % state = 2 left foot balancing
                    60  60  60, 50 50 50 50, 50 50 50 50, 80 200 100 120 200 200, 80 200 100 120 200 200]; ...     % state = 3 right foot balancing
                     
Gains.dampings   = 2*sqrt(Gains.impedances)*0;
    
%% Constraints for QP for balancing - friction cone - z-moment - in terms of f

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the 
% circle in each cicle's quadrant 
numberOfPoints               = 4; 
forceFrictionCoefficient     = 1/3;  
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10; % Min vertical force [N]

% Size of the foot
Config.footSize              = [-0.05  0.12 ;    % xMin, xMax
                                -0.040 0.040 ];   % yMin, yMax
    