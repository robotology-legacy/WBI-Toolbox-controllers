ROBOT_DOF = 23;
CONFIG.ON_GAZEBO = true;

WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';                         
                                                                                                
PORTS.IMU                 = '/icubSim/inertial'; 
PORTS.WBDT_LEFTLEG_EE     = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE    = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';

%Maximum torque value sent to actuators
sat.torque            = 60;

%Maximum variation of torque from one time step to the next
%Required as parameter of QP, but used only if
%CONFIG.QP.USE_CONTINUITY_CONSTRAINTS = true;
sat.torqueDot         = inf*ones(ROBOT_DOF,1);

%Maximum value used as upper bound constraint
sat.unboundedConstant = 1e14;

%% %%%%%%%%%%%%%%%%    Gains for desired values computation

reg.pinvDampVb      = 1e-7; %Regularizing term for matrix pseudoinverse operation in base velocity computation

%% %%%%%%%%%%%%%%%%    Controller gain parameters 
gain.weightPostural         = 0.3;
gain.weightTasks            = 100;
gain.weightMinTorques       = 1e-4;
gain.weightMinContactForces = 0;

%% %%%%%%%%%%%%%%%%    Friction cone parameters

numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each circle's quadrant 
forceFrictionCoefficient     = 1; %1/4;
torsionalFrictionCoefficient = 2/150;
gain.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];    % yMin, yMax    
fZmin                        = 10;
[ConstraintsFeetMatrix,upperBoundFeetConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);             

clear numberOfPoints forceFrictionCoefficient torsionalFrictionCoefficient gain.footSize fZmin;