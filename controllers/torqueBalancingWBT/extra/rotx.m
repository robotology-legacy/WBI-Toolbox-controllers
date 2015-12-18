function R = rotx(angleInDegs)
   alpha = angleInDegs / 180 * pi;
   R = zeros(3, 3);
   R(1,1) = 1;
   R(2,2) = cos(alpha);
   R(2,3) = -sin(alpha);
   R(3,2) = sin(alpha);
   R(3,3) = cos(alpha); 
end
