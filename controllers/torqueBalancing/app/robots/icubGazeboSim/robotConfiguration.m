% Collects all robot specific parameters which are in common between all
% demos and load them
ROBOT_DOF        = 23;
CONFIG.ON_GAZEBO = true;
PORTS.IMU        = '/icubSim/inertial';

CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];


WBT_wbiList = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch,r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';
WBT_robotName = 'icubSim';

dump.left_wrench_port  = '/icubSim/left_foot/analog:o';
dump.right_wrench_port = '/icubSim/right_foot/analog:o';

references.smoothingTimeMinJerkComDesQDes = 3.0;
sat.torque = 60;

ROBOT_DOF_FOR_SIMULINK           = eye(ROBOT_DOF);
gain.qTildeMax                   = 20*pi/180;
postures                         = 0;  
gain.SmoothingTimeImp            = 1; 

%% Constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

torsionalFrictionCoefficient = 2/150;

% physical size of foot
phys.footSize                = [ -0.07 0.07;    % xMin, xMax
                                 -0.03 0.03 ];  % yMin, yMax    
                             
fZmin                        = 10;
