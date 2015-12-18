function R = roty(angleInDegs)
   alpha = angleInDegs / 180 * pi;
   R = zeros(3, 3);
   R(2,2) = 1;
   R(1,1) = cos(alpha);
   R(1,3) = sin(alpha);
   R(3,1) = -sin(alpha);
   R(3,3) = cos(alpha); 
end
