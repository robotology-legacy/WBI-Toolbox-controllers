PORTS.WBDT_RIGHTLEG_EE  = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';

WBT_wbiList           = '(r_hip_pitch,r_knee)';

ROBOT_DOF             = 2;
ROBOT_DOF_SIM         = eye(ROBOT_DOF);

%Maximum joint torque value
sat.torque            = 60;

%When set to true, variable change for joint limit avoidance is applied
model.variableChange = true;

%Define which kind of trajectory to be applied at each joint
%   1 - ramped reference trajectory, towards max or min joint limit
%   2. sinusoidal reference trajectory
%   3. Constant arbitrary position
model.trajectory = [2;2];

%When set to true, an external torque Text will be applied at the joints 
%in the interval of time from TextIniTime to TextEndTime.
model.applyExternalTorque = false;

%% Control gains
model.Kp = [70; 20];
model.Kd = 0 * 2*sqrt(model.Kp);

%% Trajectory parameters

%1. ramped reference trajectory, towards max or min joint limit
% model.trajectory = 1;
model.ratio = 0.1 * ones(ROBOT_DOF,1);
model.ramp = [-1;1]; %ramp ratio, if negative go towards minimum

%2. sinusoidal reference trajectory
% model.trajectory = 2;
model.ratioAmplitude = 1.1 * ones(ROBOT_DOF,1);
model.rFrequency     = [pi/12; pi/4]; 
model.rPhase         = [0; -pi/2];

%3. Constant arbitrary position
model.jointPosDes = [80; -60] * pi/180;

%% External Torque parameters
%Apply an external torque Text at the motor, in the interval of time
%from TextIniTime to TextEndTime.
model.Text = 0*20 * model.ramp;
model.TextIniTime = 5;
model.TextEndTime = 5.5;
