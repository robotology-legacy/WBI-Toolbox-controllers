clear all;
clc;

n             = 25; % Number of DOF
k             = 12; % Number of constraints
numberOfTasks = 2;  % Without counting postural and the force task 

% Dimensions of tasks
m  = [5 7];

%Mass matrix and coriolis terms in the dynamics equation
%
%  M(q) dot(v) + h(q,v) - Jc^t f = S tau
 
M = rand(n + 6);
h = rand(n + 6,1);



%Desired acceleration for each tesk star{n} is the fStar
star     = cell(numberOfTasks+2,1); 
% star is a cell array that contains
% star{1} = postural desired acceleration
% star{2} = acceleration of the 2nd lowest pripority task
% ...
% star{numberOfTasks+1} = acceleration of the 2nd highest pripority task 
% star{end}             = desired force

%Then, we have the associated Jacobians and JDotqDot

J        = cell(numberOfTasks+2,1);
JDotqDot = cell(numberOfTasks+2,1);

star{1}     =  rand(n,1);  %Lowest priority for postural task
J{1}        = eye(n); 
JDotqDot{1} = zeros(n,1);
for i = 1:numberOfTasks
    star{i+1}     = rand(m(i),1);
    J{i+1}        = rand(m(i),n+6); 
    JDotqDot{i+1} = rand(m(i),1);
end
star{end}     = rand(k,1); %HIghest priority for force task
J{end}        = rand(k,n + 6);
JDotqDot{end} = rand(k,1);

% Call to SOT
tau = completeForceControlWithSubTasks(M,h,star,JDotqDot,J)

