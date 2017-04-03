ROBOT_DOF = 23;
CONFIG.ON_GAZEBO = true;

WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';


CONFIG.SMOOTH_DES_COM = 0;    % If equal to one, the desired streamed values 
                              % of the center of mass are smoothed internally 
CONFIG.SMOOTH_DES_Q   = 0;    % If equal to one, the desired streamed values 
                              % of the postural tasks are smoothed internally 
                                                                                                
PORTS.IMU = '/icubSim/inertial'; 

sat.torque          = 60;
sat.torqueDot       = 100*ones(ROBOT_DOF,1);


%% %%%%%%%%%%%%%%%%    Gains for desired values computation

reg.pinvDampVb      = 1e-7; %Regularizing term for matrix pseudoinverse operation in base velocity computation

%% %%%%%%%%%%%%%%%%    Gains for desired Task Acceleration computation

gain.CoM.posPD         = [50 * ones(11, 3), 2 * sqrt(ones(11,3))];  %for 11 states, x-y-z directions

gain.root.rotPD        = ones(11,1) * [5, 2 ];

gain.lFoot.posPD       = [25*ones(11,3), 2*sqrt(ones(11,3))]; %for 11 states, x-y-z directions
gain.lFoot.rotPD       = ones(11,1) * [25, 2 ];

gain.rFoot.posPD       = gain.lFoot.posPD;
gain.rFoot.rotPD       = gain.lFoot.rotPD;

gain.joints.impedances = 10 * ones(11, ROBOT_DOF); %for 11 states, 23 DOF
gain.joints.dampings   = 2 * sqrt(gain.joints.impedances);

%% %%%%%%%%%%%%%%%%    Controller gain parameters 
gain.weightPostural         = 0.3;
gain.weightTasks            = 100;
gain.weightMinTorques       = 1e-4;
gain.weightMinContactForces = 0;

%% %%%%%%%%%%%%%%%%    Inverse kinematics gains
ikin.CoM.posPD      = [50 * ones(3, 1)', 2*sqrt(50*ones(3,1))']; 

ikin.root.rotPD     = [50, 2*sqrt(50)];

ikin.lFoot.posPD    = [25*ones(1,3), 2*sqrt(25*ones(1,3))];
ikin.lFoot.rotPD    = [25, 2*sqrt(25)];

ikin.rFoot.posPD    = ikin.lFoot.posPD;
ikin.rFoot.rotPD    = ikin.lFoot.rotPD;

ikin.impedances     = diag(gain.joints.impedances(1,:));
ikin.dampings       = 2 * sqrt(ikin.impedances);

%% %%%%%%%%%%%%%%%%    Friction cone parameters

numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each circle's quadrant 
forceFrictionCoefficient     = 1/4; 
torsionalFrictionCoefficient = 2/150;
gain.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];    % yMin, yMax    
fZmin                        = 10;
[ConstraintsFeetMatrix,upperBoundFeetConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);             