ROBOT_DOF = 23;
CONFIG.ON_GAZEBO = false;

WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';


CONFIG.SMOOTH_DES_COM = 0;    % If equal to one, the desired streamed values 
                              % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q   = 0;    % If equal to one, the desired streamed values 
                              % of the postural tasks are smoothed internally 
                                                                                                
PORTS.IMU = '/icub/inertial'; 

PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
                            
PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';

sat.torque          = 60;
sat.torqueDot       = 100*ones(ROBOT_DOF,1);


%% %%%%%%%%%%%%%%%%    Gains for desired values computation

reg.pinvDampVb      = 1e-7; %Regularizing term for matrix pseudoinverse operation in base velocity computation

%% %%%%%%%%%%%%%%%%    Gains for desired Task Acceleration computation

gain.rootPD         = [5, 2 ];

gain.lFoot.posPD    = [25*ones(3,1), 2*sqrt(ones(3,1))];
gain.lFoot.rotPD    = [25, 2 ];

gain.rFoot.posPD    = [50*ones(3,1), 2*sqrt(ones(3,1))]; %gain.lFoot.posPD;
gain.rFoot.rotPD    = [200, 2]; %gain.lFoot.rotPD;

%% %%%%%%%%%%%%%%%%    Controller gain parameters

gain.PCOM           = 50 * ones(11, 3); %for 11 states
gain.DCOM           = 2 * sqrt(gain.PCOM);  

gain.impedances     = 10 * ones(11, ROBOT_DOF); %for 11 states
gain.dampings       = 2 * sqrt(gain.impedances);

gain.weightPostural         = 0.1;
gain.weightTasks            = 200;
gain.weightMinTorques       = 1e-4;
gain.weightMinContactForces = 1e-7;

%% %%%%%%%%%%%%%%%%    Friction cone parameters

numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each circle's quadrant 
forceFrictionCoefficient     = 1/4; 
torsionalFrictionCoefficient = 2/150;
gain.footSize                = [ -0.07  0.12   ;   % xMin, xMax
                                 -0.045 0.05 ];    % yMin, yMax  
                             
fZmin                        = 10;
[ConstraintsFeetMatrix,upperBoundFeetConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);             