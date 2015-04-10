function [Aineq_fcone_f,bineq_fcone_f]=constraint_fcone_QP(staticFrictionCoefficient,numberOfPoints,n_constraint)
%compute friction cones contraints

%approximation with straight lines

%split the pi/2 angle into numberOfPoints - 1;
segmentAngle = pi/2 / (numberOfPoints - 1);

%define angle
angle = 0 : segmentAngle : (2 * pi - segmentAngle);
points = [cos(angle); sin(angle)];
numberOfEquations = size(points, 2);
assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));

%Aineq*x <= b, with b is all zeros.
Aineq = zeros(numberOfEquations, 6);

%define equations
for i = 1 : numberOfEquations
   firstPoint = points(:, i);
   secondPoint = points(:, rem(i, numberOfEquations) + 1);
   
   %define line passing through the above points
   angularCoefficients = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
   
   offsets = firstPoint(2) - angularCoefficients * firstPoint(1);

   inequalityFactor = +1;
   %if any of the two points are between pi and 2pi, then the inequality is
   %in the form of y >= m*x + q, and I need to change the sign of it.
   if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)
       inequalityFactor = -1;
   end
   
   %a force is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
   %I have constraints on fy and fz, and the offset will be multiplied by
   %mu * fx
   
   Aineq(i,:) = inequalityFactor .*  [-angularCoefficients, 1, -offsets * staticFrictionCoefficient, 0, 0, 0];

end

switch n_constraint
    case 1
        bineq = zeros(size(Aineq,1), 1);
    case 2    
        %duplicate the matrices and vector for the two feet
        Aineq = [Aineq, zeros(size(Aineq));
                zeros(size(Aineq)), Aineq];
          
        bineq = zeros(size(Aineq,1), 1);
    otherwise
end

Aineq_fcone_f = Aineq;
bineq_fcone_f = bineq;

end