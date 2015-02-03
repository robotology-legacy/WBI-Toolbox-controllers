function [tau,CoMError]  =  fcn(q,qjInit,v, M, h, g, H, PosRightFoot, Jc, JcDqD, pos_CoM,J_CoM, Desired_x_dx_ddx_CoM, Gains, Impedances, IntErrorCoM)

% t...
    

n_dof = 25;
n_constraint=2;

PINV_TOL                = 1e-5;
gravAcc                 = 9.81;

m= M(1,1);
St    = [zeros(6,n_dof);eye(n_dof,n_dof)];
grav = [-m*gravAcc;zeros(2,1);zeros(3,1)];
pos_rightFoot = PosRightFoot(1:3);
JcMinv        = Jc/M;
JcMinvSt = JcMinv*St;
JcMinvJct = JcMinv*transpose(Jc);

% to_store = [prm.joint_limits(:,1),q,prm.joint_limits(:,2)];
% joints_vs_lim = (to_store(:,2)-to_store(:,1))./(to_store(:,3)-to_store(:,1));
% joints_above_limits = joints_vs_lim>=1 | joints_vs_lim<=0;

% if isempty(to_store(joints_above_limits))
% else
% %     clc
% %     disp('joint limits reached!');
% %     [to_store , joints_above_limits]
%     
% end
% [to_store , joints_above_limits]


xDcom      = J_CoM(1:3,:)*v;
xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(pos_CoM - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));

Pr =  pos_rightFoot - pos_CoM; % Application point of the contact force on the right foot w.r.t. CoM

switch n_constraint
    case 1 % on left foot
        A  = [ eye(3),   zeros(3);
             -Sf(pos_CoM),  eye(3)];
    case 2 % on both feet
        A  = [ eye(3),   zeros(3),eye(3), zeros(3);
             -Sf(pos_CoM),  eye(3), Sf(Pr), eye(3) ];
    otherwise
        disp('Choose number of constraints properly (1 or 2)');
        return
end        

pinvA         = pinv(        A, PINV_TOL);
PInv_JcMinvSt = pinv( JcMinvSt, PINV_TOL);
HDotDes       = [ m*xDDcomStar ;
                -Gains(4)*H(4:end)]; 
         
%% QP FOR TORQUE MINIMIZATION BASED ON arbitrary vector F0

% options = optimset('Algorithm','active-set','Display','off');

%define tau_0 (the arbitrary vector)
mult_f_tau0     = -transpose(Jc(:,7:end)); % multiplier of f in tau0
n_tau0          = g(7:end) - (Impedances.*(q-qjInit)); % additional terms in tau0
%

N0_tau          = eye(n_dof) - PInv_JcMinvSt*JcMinvSt;
mult_f_tau      = PInv_JcMinvSt*(-JcMinvJct)+N0_tau*mult_f_tau0;
n_tau           = PInv_JcMinvSt*(JcMinv*h-JcDqD)+N0_tau*n_tau0;

% f desired is given as:
% f*   = pinvA*(HDotDes - grav)  + (I-pinvA*A) * f0
% f*   =           c_f           +     N0_f    * f0
N0_f            = eye(6*n_constraint,6*n_constraint)-pinvA*A;
c_f             = pinvA*(HDotDes - grav);

% tau is given as:
% tau = (mult_f_tau*N0_f)* f0 + (n_tau+mult_f_tau*c_f)
% tau =        X_tau     * f0 +                   Y_tau

X_tau = mult_f_tau*N0_f;
Y_tau = n_tau+mult_f_tau*c_f;
quadraticTerm2 = 2*(transpose(X_tau)*X_tau)+eye(6*n_constraint)*1e-8;
linearTerm2 = transpose(2*X_tau'*Y_tau);

% CONSTRAINTS

    %inequalities
    
        %friction
staticFrictionCoefficient = 2;
numberOfPoints = 4; %number of points in a quadrant for cone

[Aineq_fcone_f,bineq_fcone_f] = constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint);

Aineq = Aineq_fcone_f*N0_f;
bineq = bineq_fcone_f-Aineq_fcone_f*c_f;
    
        %moments
        
%add constraints on moments

% Aineq_moment_f = [0, 0, 0, 0, 1, 0,zeros(1,6);
%         0, 0, 0, 0,-1, 0,zeros(1,6);
%         0, 0, 0, 0, 0, 1,zeros(1,6);
%         0, 0, 0, 0, 0,-1,zeros(1,6);
%         zeros(1,6), 0, 0, 0, 0, 1, 0;
%         zeros(1,6), 0, 0, 0, 0,-1, 0;
%         zeros(1,6), 0, 0, 0, 0, 0, 1;
%         zeros(1,6), 0, 0, 0, 0, 0,-1];
% bineq_moment_f = 2*[5;5;5;5;5;5;5;5];
% 
% Aineq_moment_f0 = Aineq_moment_f*N0_f;
% bineq_moment_f0 = bineq_moment_f-Aineq_moment_f*c_f;
% 
% Aineq = [Aineq;Aineq_moment_f0];
% bineq = [bineq;bineq_moment_f0];
        





% Aeq = JcMinv*St*X_tau + JcMinvJct*(eye(12,12)-pinvA*A);
% beq = JcMinv*h - JcDqD - JcMinv*St*Y_tau - JcMinvJct*c_f;


% INITIAL CONDITION
x0 = zeros(6*n_constraint,1);

% BOUNDS ON F0
lb = [];
ub = [];

% [desiredf0, objVal, exitFlag, output, lambda] = ... 
[desiredf0, ~, ~, ~, ~] = ...
quadprog(quadraticTerm2, linearTerm2, ...
          Aineq, bineq, ... %inequalities
          [], [], ... %equalities
          lb, ub, ... %bounds
          x0,     ... %initial solution
          'Algorithm','active-set','Display','off');
%           options);
% clc
% % objVal+0.5*(grav-HDotDes)'*(grav-HDotDes)
% exitFlag
if exitFlag~=1
    disp('qp_tau_f0 failed');
end
% % output
% % [x0 , desiredFeetContactForces]






%%
% desiredFeetContactForces = pinvA*(HDotDes-grav); % to ignore QP
% 
% measuredFeetContactForces = desiredFeetContactForces;
% 
% tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinvJct*measuredFeetContactForces);
% tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances.*(q-qjInit));
% 
% tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;

tau = X_tau*desiredf0+Y_tau;

CoMError    = pos_CoM - Desired_x_dx_ddx_CoM(:,1);


normCoMError  = norm(CoMError);
end




function [Aineq,bineq]=constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint)
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
Aineqtemp = zeros(numberOfEquations, 6);

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
   
   Aineqtemp(i,:) = inequalityFactor .* [-offsets * staticFrictionCoefficient,-angularCoefficients, 1, 0, 0, 0];  
   
end

switch n_constraint
    case 1
        bineq = zeros(size(Aineqtemp,1), 1);
    case 2    
        %duplicate the matrices and vector for the two feet
        Aineq = [Aineqtemp, zeros(size(Aineqtemp));
                zeros(size(Aineqtemp)), Aineqtemp];
          
        bineq = zeros(size(Aineq,1), 1);
    otherwise
end

end

% % % 
% % % PINV_TOL = 1e-5;
% % % gravAcc  = 9.81;
% % % 
% % % n = 25;
% % % St    = [zeros(6,n);eye(n,n)];
% % % 
% % % m     = M(1,1);
% % % Mb    = M(1:6,1:6);
% % % Mbj   = M(1:6,7:end);
% % % 
% % % grav = [zeros(2,1);-m*gravAcc;zeros(3,1)];
% % % 
% % % % Definition of reference accelerations for CoM and Joints that ensure
% % % % stabilization of desired trajectories xDcomDes and qDes 
% % % 
% % % xDcom      = Jcom(1:3,:)*qD;
% % % xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(xcom - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));
% % % 
% % % Pr =  PosRightFoot(1:3) - xcom; % Application point of the contact force on the right foot w.r.t. CoM
% % % 
% % % A  = [ eye(3),   zeros(3),eye(3), zeros(3);
% % %       -Sf(xcom),  eye(3), Sf(Pr), eye(3) ];
% % % pinvA = pinv(A, PINV_TOL);
% % %   
% % % HDotDes  = [ m*xDDcomStar ;
% % %             -Gains(4)*H(4:end)]; 
% % % 
% % % Jc    =   feetJacobians;
% % %            
% % % JcDqD = feetJdqd;
% % %             
% % % desiredFeetContactForces = pinvA*(HDotDes-grav);    
% % % 
% % % JcMinv        = Jc/M;
% % % PInv_JcMinvSt = pinv(JcMinv*St, PINV_TOL);
% % % N0            = eye(n) - PInv_JcMinvSt*JcMinv*St;
% % % 
% % % measuredFeetContactForces = desiredFeetContactForces;
% % % 
% % % tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinv*Jc'*measuredFeetContactForces);
% % % 
% % % tauForImpedenceBehaviour = - Impedances.*(q-qInit) + g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - Mbj'/Mb*(h(1:6) - Jc(:,1:6)'*measuredFeetContactForces);
% % % 
% % % tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;
% % % 
% % % CoMError    = xcom - Desired_x_dx_ddx_CoM(:,1);
% % % % normCoMError  = norm(CoMError);
% % % 
% % % 
