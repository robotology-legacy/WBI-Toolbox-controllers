function [a,b,c,R] = vecs2euler(v_grav,v_acc)

% TRES = 
% definitions:
% v_grav = R*v_acc
% R= R(a,b,c) = Rz(a)Ry(b)Rz(c)
% R= [cos(a)*cos(b)*cos(c) - sin(a)*sin(c) , -cos(a)*cos(b)*sin(c) - sin(a)*cos(c) ,  cos(a)*sin(b); 
%     sin(a)*cos(b)*cos(c) + cos(a)*sin(c) , -sin(a)*cos(b)*sin(c) + cos(a)*cos(c) ,  sin(a)*sin(b);
%                          - sin(b)*cos(c) ,                         sin(b)*sin(c) ,         cos(b)];

% rotation axis between the two vectors
v_rot = cross(v_acc,v_grav);
axis_rot = v_rot/norm(v_rot);

% sine of the angle of rotation
ang_sin = norm(v_rot)/(norm(v_grav)*norm(v_acc)); 

%cosine of the angle of rotation
ang_cos = v_grav'*v_acc/(norm(v_grav)*norm(v_acc));

% using Rodrigues' formula ():
R = eye(3,3) + skew(axis_rot)*ang_sin + skew(axis_rot)*skew(axis_rot)*(1-ang_cos);

b = atan2(  sqrt(R(3,1)*R(3,1) + R(3,2)*R(3,2)) ,       R(3,3)   );
a = atan2(               R(2,3)/sin(b)          ,   R(1,3)/sin(b));
c = atan2(               R(3,2)/sin(b)          ,  -R(3,1)/sin(b));
