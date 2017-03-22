function [T_bar,angles] = fromRotMatr_toZYXEulerAngles(rot_matr)
%% fromRotMatr_toZYXEulerAngles
%  Computes the Z-Y-X Euler angles parametrization of rotations from a 
%  given rotation matrix. The outputs are:
%  
%  T_bar    [3x3] the matrix which converts the angular velocity into the
%                 Euler angles time derivative
%
%  angles   [3x1] Z-Y-X Euler angles
%

%% Euler angles conversion
phi   =  atan2(rot_matr(3,2),rot_matr(3,3));
theta = -asin(rot_matr(3,1));
psi   =  atan2(rot_matr(2,1),rot_matr(1,1));

angles = [phi theta psi];

%Matrix which links the angular velocity with the time derivative of Euler
%angles

T_bar = [1     0           -sin(theta);
         0  cos(phi)   sin(phi)*cos(theta) ;
         0 -sin(phi)   cos(phi)*cos(theta)];
         
end