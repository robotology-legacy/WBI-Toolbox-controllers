ROBOT_DOF = 1;
CONFIG.ON_GAZEBO = true;
Ts = CONFIG.Ts;

%Trajectory tracking gains
model.Kpx = 0.1;
model.Kdx = 0.1;

%Joint limit avoidance Parameters 
%(decelerate when entering a critical area close to joint limits)
model.mu_p = 1; %1.7; % >0, Intensity coefficient (magnitude of deceleration in critical area. The higher, the less deceleration. 0 means MUCH deceleration)
model.eta_p = 1; % (0,1), Critical coefficient (defines the critical areas in which deceleration takes place)

%Control gains
model.Kp = 25 * eye(ROBOT_DOF);
model.Kd = 0.5 * eye(ROBOT_DOF);
model.Ki = 5 * eye(ROBOT_DOF);


%% Trajectory parameters

%ramped reference trajectory, towards max or min joint limit
% model.trajectory = 1;
model.rmin = -2.1817 * 0.9; %minimum reference position
model.rmax = 0.4014 - 0.1; %* 0.9; %maximum reference position
model.p = 0.1; %-1;%1; %ramp ratio, if negative go towards minimum
model.r0 = -10.36 * pi / 180; %-0.2; %initial reference position

%sinusoidal reference trajectory
% model.trajectory = 2;
%knee joint limits -2.1817:0.4014
model.qo = (-2.1817+0.4014)/2;
model.delta = (0.4014+2.1817)/2;
model.rAmplitude = model.delta *0.9;
model.rFrequency = pi/4;
model.rbias = model.qo;

%choose which trajectory from the above
model.trajectory = 1;

%% External Torque parameters
model.Text = 0 * 15 * model.p;
model.TextIniTime = 4;
model.TextEndTime = 4.5;
