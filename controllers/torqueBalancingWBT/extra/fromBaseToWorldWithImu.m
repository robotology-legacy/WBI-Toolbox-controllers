function w_H_root = fromBaseToWorldWithImu(imu_H_link,link_H_root,inertial)
%#codegen
 

% See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 11

w_R_imu    = rotz(inertial(3))*roty(inertial(2))*rotx(inertial(1));

imu_R_link = imu_H_link(1:3,1:3);


w_R_link   = w_R_imu*imu_R_link;

w_H_link   = [w_R_link,zeros(3,1)
              zeros(1,3),1];

w_H_root   = w_H_link*link_H_root;