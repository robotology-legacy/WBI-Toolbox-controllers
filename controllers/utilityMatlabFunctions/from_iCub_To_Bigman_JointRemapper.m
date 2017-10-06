function qjRemapped = from_iCub_To_Bigman_JointRemapper(qj,ndof)
% from_iCub_To_Bigman_JointRemapper
%
% this function defines a map between two different joint position vectors:
% qj [ndof x 1] is a position vector representing the joint positions of the 
% robot Walkman, with the signs convention of RBCS lab. qjRemapped [ndof x 1]
% still represents the joint positions of Walkman robot, but signs convention
% is the one used in the LOC2 team on the robot iCub. ndof is a scalar
% representing the number of controlled robot joints. 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, August 2017
%

%% ------------Initialization----------------

% joint vector is divided into a certain number of smaller vectors, each
% one representing a robot part
torso = [qj(1) qj(2) qj(3)];

% check on the robot DoFs. Currently supported configurations: full controlled
% (25 DoF), only legs (12 DoF) and no wrist pronosupination (23 DoF)
if ndof == 23
    
    lArm  = [qj(4) qj(5) qj(6)  qj(7)];
    rArm  = [qj(8) qj(9) qj(10) qj(11)];
    lLeg  = [qj(12) qj(13) qj(14) qj(15) qj(16) qj(17)];
    rLeg  = [qj(18) qj(19) qj(20) qj(21) qj(22) qj(23)];

elseif ndof == 25

    lArm  = [qj(4) qj(5)  qj(6)  qj(7)  qj(8)];
    rArm  = [qj(9) qj(10) qj(11) qj(12) qj(13)];
    lLeg  = [qj(14) qj(15) qj(16) qj(17) qj(18) qj(19)];
    rLeg  = [qj(20) qj(21) qj(22) qj(23) qj(24) qj(25)];
    
elseif ndof == 12

    lLeg  = [qj(1) qj(2) qj(3) qj(4) qj(5) qj(6)];
    rLeg  = [qj(7) qj(8) qj(9) qj(10) qj(11) qj(12)];

end

%% Remapping according to the iCub signs convention

% define the vector of remapped joint positions
qjRemapped = [];

if ndof == 23 || ndof == 25
    
    % torso: [pitch roll yaw]
    torso(2) = -torso(2);
    torso(3) = -torso(3);

    % lArm: [sh_pitch sh_roll sh_yaw elbow forearm]
    lArm(3)  = -lArm(3);
    lArm(4)  = -lArm(4);

    if ndof == 25
     lArm(5)  = -lArm(5);
    end

    % rArm: [sh_pitch sh_roll sh_yaw elbow forearm]
    rArm(2)  = -rArm(2);
    rArm(4)  = -rArm(4);

    % lLeg: [hip_pitch hip_roll hip_yaw knee ankle_pitch ankle_roll]
    lLeg(1)   = -lLeg(1);
    lLeg(4)   = -lLeg(4);

    % rLeg: [hip_pitch hip_roll hip_yaw knee ankle_pitch ankle_roll]
    rLeg(1)   = -rLeg(1);
    rLeg(2)   = -rLeg(2);
    rLeg(3)   = -rLeg(3);
    rLeg(4)   = -rLeg(4);
    rLeg(6)   = -rLeg(6);

    % new joint positions (23-25 Dofs)
    qjRemapped = [torso,lArm,rArm,lLeg,rLeg];

elseif ndof == 12

    % lLeg: [hip_pitch hip_roll hip_yaw knee ankle_pitch ankle_roll]
    lLeg(1)    = -lLeg(1);
    lLeg(4)    = -lLeg(4);

    % rLeg: [hip_pitch hip_roll hip_yaw knee ankle_pitch ankle_roll]
    rLeg(1)    = -rLeg(1);
    rLeg(2)    = -rLeg(2);
    rLeg(3)    = -rLeg(3);
    rLeg(4)    = -rLeg(4);
    rLeg(6)    = -rLeg(6);

    % new joint positions (12 Dofs)
    qjRemapped = [lLeg,rLeg];
    
end
end






