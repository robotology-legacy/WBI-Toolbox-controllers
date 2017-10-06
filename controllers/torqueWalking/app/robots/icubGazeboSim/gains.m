CONFIG.ON_GAZEBO = true;

ROBOT_DOF = 23;
WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';                         
                                                                                                
PORTS.IMU                 = '/icubSim/inertial'; 
PORTS.WBDT_LEFTLEG_EE     = '/wholeBodyDynamicsTree/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE    = '/wholeBodyDynamicsTree/right_leg/cartesianEndEffectorWrench:o';


%% Constants used for tolerance/saturation

constants.maxTolerance     = 1e14; %Maximum value for unconstrained variable
constants.minTolerance     = 1e-4; %Tolerance on the value of an equality constrained variable
constants.saturationTorque = 60;   %Maximum torque value sent to actuators
constants.saturationForce  = 500; %Maximum contact force value considered

reg.pinvDampVb             = 1e-10; %Regularizing term for matrix pseudoinverse operation in base velocity computation

%Maximum variation of torque from one time step to the next
%Required as parameter of QP, but used only if CONFIG.QP.USE_CONTINUITY_CONSTRAINTS = true;
%i.e. not used at the moment
sat.torqueDot       = inf*ones(ROBOT_DOF,1);

%% Controller gains

%Center of mass linear proportional (p) and derivative (d) gains
gain.x_CoM.p = 60;
gain.x_CoM.d = 2 * sqrt(gain.x_CoM.p);

%Center of mass angular proportional (p) and derivative (d) gains
gain.w_CoM.p  = 5;
gain.w_CoM.d  = 2 * sqrt(gain.w_CoM.p);

%Joints proportional (p) and derivative (d) gains
gain.joints.p = 20;
gain.joints.d = 2*sqrt(gain.joints.p);

%Weight on regularization of joint torques
gain.reg.joint_torques = 1e-7;

%Factor multiplying CoM gains, for CoM acceleration bounds
%value [0;1]
gain.CoMboundFactor = 0.1;


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