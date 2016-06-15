%gravity
model.g = 9.81;

%Init robot model, 2 links
%Number of DOFs of the system
model.n = 2;
ROBOT_DOF = model.n;
ROBOT_DOF_SIM = eye(ROBOT_DOF);

%Link 1
model.m1 = 2; %mass
model.a1 = 1.5; %length
model.l1 = 0.75; %distance from joint to CoM
model.Il1 = 0.2528; %inertia around CoM

%Link 2
model.m2 = 2; %mass
model.a2 = 1.5; %length
model.l2 = 0.75; %distance from joint to CoM
model.Il2 = 0.2528; %inertia around CoM 

%friction coefficient
model.Fv = eye(2)*0;

%Joint limits //assume both joints are the same
model.qmin = -pi * ones(model.n,1); %used to be 0
model.qmax = pi * ones(model.n,1);
model.qo = (model.qmax + model.qmin)/2;
model.delta = (model.qmax - model.qmin)/2;

% % initial conditions
q0 = zeros(model.n, 1); %[0.01; 0.01]; %zeros(2,1);
qd0 = zeros(model.n, 1);
% % %random initial conditions and trajectory
% % q0 = rand(2,1)*(model.qmax-0.1)+(model.qmin + 0.1);
% % qd0 = randn(2,1);
% % r = rand(2,1)*(model.qmax-0.15)+(model.qmin + 0.15);



%% Trajectory parameters
%1. ramped reference trajectory, towards max or min joint limit
% model.trajectory = 1;
model.rmin = model.qmin * 0.9; %minimum reference position
model.rmax = model.qmax * 0.9; %maximum reference position
model.p = 1 * [1; 1]; %-1; %ramp ratio, if negative go towards minimum
model.r0 = model.qmax * 0.5; %model.rmax; %-0.2; -10.36 * pi/180; % %initial reference position
% Reference trajectory
r = model.r0;

%2. sinusoidal reference trajectory
% model.trajectory = 2;
model.rAmplitude = [0; model.delta] * 0.85;
model.rFrequency = [0; pi/4];
model.rbias = [model.r0(1); model.qo(2)];

%choose which trajectory from the above
model.trajectory = 2;

%% External Torque parameters
%Apply an external torque Text at the motor, in the interval of time
%from TextIniTime to TextEndTime.
model.Text = 25 * model.p;
model.TextIniTime = 10;
model.TextEndTime = 10.5;


%Gravity Compensation
model.Kp = 15 * [1 0; 0 1];
model.Kd = 5 * sqrt(model.Kp);