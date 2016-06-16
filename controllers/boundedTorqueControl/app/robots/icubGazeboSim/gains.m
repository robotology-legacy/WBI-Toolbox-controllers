ROBOT_DOF = 1;
ROBOT_DOF_SIM = eye(ROBOT_DOF);
CONFIG.ON_GAZEBO = true;

model.noVariableChange = true;

%Control gains
K = 2;
model.Kp = K * ones(ROBOT_DOF,1);
model.Kd = 2*sqrt(model.Kp);

% %Control gains for computed torque control
% %Uncomment if using this control
% KCTC = 30;
% model.KpCTC = KCTC * eye(ROBOT_DOF);
% model.KdCTC = 0.5 * sqrt(model.KpCTC);
% model.KiCTC = 1 * model.KdCTC;

%knee joint limits [-2.1817,0.4014]
model.qo    = (-2.1817+0.4014)/2;
model.delta = (0.4014+2.1817)/2;


%% Trajectory parameters

%1. ramped reference trajectory, towards max or min joint limit
% model.trajectory = 1;
model.rmin = -2.1817 * 0.9; %minimum reference position
model.rmax = 0.4014 - 0.1; %maximum reference position
model.p = 1; %-1; %ramp ratio, if negative go towards minimum
model.r0 = model.rmax; %-0.2; -10.36 * pi/180; % %initial reference position


%2. sinusoidal reference trajectory
% model.trajectory = 2;
model.ratioAmplitude = 1.1; % 0.95;
model.rFrequency     = 0.1;
model.rbias          = model.qo;

%choose which trajectory from the above
model.trajectory = 2;

%% External Torque parameters
%Apply an external torque Text at the motor, in the interval of time
%from TextIniTime to TextEndTime.
model.Text = 20 * model.p;
model.TextIniTime = 5;
model.TextEndTime = 5.5;




%% 23 DOF ROBOT:

% K = 50;
% KpTorso   = 2*ones(1,3); %0.5*ones(1,3);
% KpArms    = ones(1,4); %[0.1,0.1,0.1,0.1]; 
% KpLegs    = ones(1,6); %[0.5,0.5,0.5,0.5,0.5,0.5]; [0.8,1,1,0.8,2,1]; %
% Kp        = K*diag([KpTorso,KpArms,KpArms,KpLegs,KpLegs]);
% 
% Kd        = 2*sqrt(Kp);
% Ki        = Kd*3;


%reference position is the homing position
%torso (1,2,3)
%Larm shoulder (4,5,6) elbow (7)
%Rarm shoulder (8,9,10) elbow (11)
%Lleg hip (12,13,14) knee (15) ankle (16,17)
%Rleg hip (18,19,20) knee (21) ankle (22,23)

% % r = [-0.0524    0.0000   -0.0000   ...                                %1 2 3
% %      -0.6278    0.5231    0.0010    0.8727   ...                      %4 5 6 7
% %      -0.6278    0.5231    0.0010    0.8727    ...                     %8 9 10 11
% %       0.2094    0.0873   -0.0000   -0.1746   -0.0489   -0.0873    ... %12 13 14 15 16 17
% %       0.2096    0.0872   -0.0000   -0.1745   -0.0279   -0.0698]';     %18 19 20 21 22 23
 
% lim = [-0.3840:1.4661   -0.6807:0.6807   -1.0297:1.0297   ... 
%        -1.6581:0.0873    0:2.8065        -0.6458:1.7453    0.0960:1.8500   ... 
%        -1.6581:0.0873    0:2.8065        -0.6458:1.7453    0.0960:1.8500   ...
%        -0.7679:2.3038    -0.2967:2.0769   -1.3788:1.3788   -2.1817:0.4014   -0.7330:0.3665   -0.4189:0.4189  ...
%        -0.7679:2.3038    -0.2967:2.0769   -1.3788:1.3788   -2.1817:0.4014   -0.7330:0.3665   -0.4189:0.4189 ]';

% r(8) = -0.1; %right shoulder
% r(11) = 1.7; %right elbow


% qmin = [-0.3840  -0.6807   -1.0297   ... 
%        -1.6581    0        -0.6458    0.0960  ... 
%        -1.6581    0        -0.6458    0.0960  ...
%        -0.7679   -0.2967   -1.3788   -2.1817   -0.7330   -0.4189  ...
%        -0.7679   -0.2967   -1.3788   -2.1817   -0.7330   -0.4189 ]';
% qmax = [1.4661   0.6807   1.0297   ... 
%        0.0873    2.8065   1.7453    1.8500   ... 
%        0.0873    2.8065   1.7453    1.8500   ...
%        2.3038    2.0769   1.3788    0.4014   0.3665   0.4189  ...
%        2.3038    2.0769   1.3788    0.4014   0.3665   0.4189 ]';
% delta = (qmax - qmin)/2;
% q0 = (qmax + qmin)/2;

