CONFIG.ON_GAZEBO = true;

ROBOT_DOF = 23;
WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';                         
                                                                                                
PORTS.IMU              = '/icubSim/inertial'; 
PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';

%% Constants used for tolerance/regularization/saturation

constants.maxTolerance     = 1e17; %Maximum value for unconstrained variable
constants.minTolerance     = 1e-4; %Tolerance on the value of an equality constrained variable
constants.saturationTorque = 60;   %Maximum torque value sent to actuators
constants.saturationForce  = 500;  %Maximum contact force value considered
constants.thresholdContact = 10; %25;   %Minimum vertical force to consider a contact to be active

reg.pinvDamp      = 1e-10; %Regularizing term for matrix pseudoinverse operation in base velocity computation
reg.joint_torques = 1e-7;  %Weight on regularization of joint torques

%% %%%%%%%%%%%%%%%%    State Machine information
% Move to the left
root_displacement = [0; -0.15; 0];


%% Controller gains

%Root link linear proportional gains
gain.x_root.p     = 15;

%Root link angular proportional gains
gain.w_root.p     = 15;

%Joints proportional gains
gain.joints.torso = 10;
gain.joints.arms  = 30;
gain.joints.legs  = 0.005;
                 
%Gains for root acceleration bounds
gain.x_maxAcceleration = 15;
gain.x_rootbound.p     = 1;

%Root link and joints derivative gains  -- no need to touch those =)
gain.joints.p     = [gain.joints.torso * ones(3,1); 
                     gain.joints.arms  * ones(8,1); 
                     gain.joints.legs  * ones(12,1)];
gain.w_root.d     = 2 * sqrt(gain.w_root.p);
gain.x_root.d     = 2 * sqrt(gain.x_root.p);                 
gain.joints.d     = 2 * sqrt(gain.joints.p);
gain.x_rootbound.d= 2 * sqrt(gain.x_rootbound.p);

%% %%%%%%%%%%%%%%%%    Friction cone parameters

numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each circle's quadrant 
forceFrictionCoefficient     = 1; %1/4;
torsionalFrictionCoefficient = 2/150;
gain.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];    % yMin, yMax    
fZmin                        = 10;
[frictionConeConstraintsMatrix,upperBoundFrictionConeConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);             

clear numberOfPoints forceFrictionCoefficient torsionalFrictionCoefficient gain.footSize fZmin;

%Maximum variation of torque from one time step to the next
%Required as parameter of QP, but used only if CONFIG.QP.USE_CONTINUITY_CONSTRAINTS = true;
%i.e. not used at the moment
sat.torqueDot = inf*ones(ROBOT_DOF,1);