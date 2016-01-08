clear all;
clc;
a = -pi/2;
b = -a;

inertial  = a +(b-a)*rand(1,3)

w_R_imu      = rotz(inertial(3))*roty(inertial(2))*rotx(inertial(1));


rollPitchYaw = rollPitchYawFromRotation(w_R_imu);

inertial1 = rollPitchYaw'

w_R_imu1      = rotz(inertial1(3))*roty(inertial1(2))*rotx(inertial1(1));


norm(w_R_imu1-w_R_imu)