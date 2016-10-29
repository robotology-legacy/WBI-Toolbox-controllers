function [torso,leftarm,rightarm,leftleg,rightleg] = increaseDofTo25ForVisualizer(WBI_ordered,ROBOT_DOF)
%INCREASEDOFTO25FORVISUALIZER increases the number of DoFs to the current
%   maximum (25) by adding some zeros in case one or more joints are not
%   used for balancing. The reason for this is to have a unique visualizer
%   for any robot configuration (currently 12,15,23,and 25 DoFs).
%
%   [torso,leftarm,rightarm,leftleg,rightleg] = increaseDofTo25ForVisualizer
%   (WBI_ordered,ROBOT_DOF) takes as input the number of DoF of the system, and
%   any joints-related vector (e.g. joint position, acceleration,
%   velocity,...). The output are 5 vectors related to the different robot
%   parts (torso,legs,arms). 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, October 2016
%

% ------------Initialization----------------

%% Config parameters
joints   = WBI_ordered;
torso    = zeros(3,1);
leftarm  = zeros(5,1);
rightarm = zeros(5,1);
leftleg  = zeros(6,1);
rightleg = zeros(6,1);

%% Different robots configurations
if size(ROBOT_DOF,1) == 12
    
    leftleg  = joints(1:6);
    rightleg = joints(7:12);
    
elseif size(ROBOT_DOF,1)  == 15
    
    torso    = joints(1:3);
    leftleg  = joints(4:9);
    rightleg = joints(10:15);  
    
elseif size(ROBOT_DOF,1)  == 23
    
    torso         = joints(1:3);
    leftarm       = [joints(4:7);0];
    rightarm      = [joints(8:11);0];
    leftleg       = joints(12:17);
    rightleg      = joints(18:23); 
    
elseif size(ROBOT_DOF,1)  == 25
        
    torso         = joints(1:3);
    leftarm       = joints(4:8);
    rightarm      = joints(9:13);
    leftleg       = joints(14:19);
    rightleg      = joints(20:25); 
end

end