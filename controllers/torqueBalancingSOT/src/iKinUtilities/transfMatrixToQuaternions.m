function qInit = transfMatrixToQuaternions(qInitQuat)

% Consider the base pose
q_base      = qInitQuat(1:16);

% Separate the rotation matrix
R_Base      = [qInitQuat(1:3) qInitQuat(5:7) qInitQuat(9:11)];

% Define the elements of the rotation matrix
r00 = R_Base(1,1);
r11 = R_Base(2,2);
r22 = R_Base(3,3);

r21 = R_Base(3,2);
r12 = R_Base(2,3);
r02 = R_Base(1,3);
r20 = R_Base(3,1);
r10 = R_Base(2,1);
r01 = R_Base(1,2);

% Generate the quaternion
qw = sqrt(1 +r00 +r11 +r22)/2;
qx = (r21 -r12)/(4*qw);
qy = (r02 -r20)/(4*qw);
qz = (r10 -r01)/(4*qw);

quaternions = [qw; qx; qy; qz];

% Final state converted as a 7+n element vector
qInit   =  [ q_base(13:15); quaternions; qInitQuat(17:end)];

end