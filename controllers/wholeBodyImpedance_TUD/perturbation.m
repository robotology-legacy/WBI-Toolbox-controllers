%%Creation of the perturbation torque vector and application time


robotName = 'icubGazeboSim';
localName = 'simulink';
Ts = 0.1 ; 
Tt = 30 ;
Tm_t=50;  %perturbation application time


torso_pitch = 5;
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