function w_H_imu = fromImuToHomogeousTransform(inertial)
%#codegen
 

% See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 11

w_R_imu = rotz(inertial(3))*roty(inertial(2))*rotx(inertial(1));

w_H_imu = [  w_R_imu,  zeros(3,1);
           zeros(1,3),    1        ];