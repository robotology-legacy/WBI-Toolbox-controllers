%%Creation of the perturbation torque vector and application time

%%Simulation Parameters
robotName = 'icubGazeboSim';
localName = 'simulink';
Ts = 0.1 ; 

%%Controller Parameters
P=diag(25*ones(1,25));                                                      %Proportional Gain
D=diag(2*sqrt(20)*ones(1,25));                                              %Derivative Gain

%%Reference Parameters
c=0;                                                                        %Default trajectory, i.e. no vector of joint angle is provided
A=0*[20/180*pi;zeros(23,1);0/180*pi];                                       %Sine wave amplitude (vector 25x1)
f=0*1/100;                                                                  %Sine wave frequency (vector 25x1)
s=0*[10/180*pi/10;zeros(24,1)];                                             %Ramp slope (vector 25x1)
st=0*[20/180*pi;0/180*pi;0/180*pi;0/180*pi;0/180*pi;zeros(20,1)];           %Step value (vector 25x1)
get_data;                                                                   %Read the custom trajectory
%%Gaussian noise
Vq=0*0.5/180*pi;                                                            %Variance on joint position (in deg)
Vdq=0*0.02;                                                                 %Variance on joint speed (in %)
Vt=0*0.05;                                                                  %Variance on torque (in %)

%%Perturbation Torque Parameters
P_on=0;                                                                     %Perturbation activation
Tm_t=50;                                                                    %Perturbation application time
tin=[10;60;zeros(23,1)];                                                    %Time at which perturbation starts
                                                                   
torso_pitch = 2;                                                            %Value of the torque applied on each joint
torso_roll = 0;
torso_yaw =	0;
left_shoulder_pitch = 0;
left_shoulder_roll = 0;
left_shoulder_yaw = 0;
left_elbow = 0;
left_wrist_prosup = 0;
right_shoulder_pitch = 0;
right_shoulder_roll = 0;
right_shoulder_yaw = 0;
right_elbow = 0;
right_wrist_prosup = 0;
left_hip_pitch = 0;
left_hip_roll = 0;
left_hip_yaw = 0;
left_knee = 0;
left_ankle_pitch = 0;
left_ankle_roll = 0;
right_hip_pitch = 0;
right_hip_roll = 0;
right_hip_yaw = 0;
right_knee = 0;
right_ankle_pitch = 0;
right_ankle_roll = 0;

tau_p=[torso_pitch; torso_roll; torso_yaw; left_shoulder_pitch; left_shoulder_roll; left_shoulder_yaw; left_elbow; left_wrist_prosup; ...
    right_shoulder_pitch; right_shoulder_roll; right_shoulder_yaw; right_elbow; right_wrist_prosup; left_hip_pitch; left_hip_roll; ...
    left_hip_yaw; left_knee; left_ankle_pitch; left_ankle_roll; right_hip_pitch;  right_hip_roll; right_hip_yaw; right_knee; right_ankle_pitch; right_ankle_roll];