function [Qs,w_H_l_sole] = computeRotTransFromLsole2World(inertialSeesaw,seesaw)
%#codegen

    e3     = [0;0;1];
    % Converting degree into rad
    inertialSeesaw     = (inertialSeesaw   * pi)/180;

    % Composing the rotation matrix:
    % See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 12

    w_R_s  = rotz(inertialSeesaw(3))*roty(inertialSeesaw(2))*rotx(inertialSeesaw(1));
    % Quaternion associated with w_R_s
    Qs     = quaternionFromRotationMatrix(w_R_s);

    % s_s_l = positionOfLeftFoot - seesawCoM w.r.t. seesaw frame
    s_s_l     = [0;  seesaw.lFootDistanceCenter; seesaw.top];

    % OC = originOfWorld - contactPointSeesaw
    OC = [ 0;
          -seesaw.rho*inertialSeesaw(1);
           seesaw.rho]; 

    % CL = positionOfLeftFoot - centerOfRotationSeesaw w.r.t. world frame

    CL = w_R_s*(s_s_l-seesaw.delta*e3);    

    % P_l = positionOfLeftFoot w.r.t. world frame
    P_l = OC + CL;
    w_H_l_sole = [w_R_s,     P_l;
                  zeros(1,3),  1];
end
