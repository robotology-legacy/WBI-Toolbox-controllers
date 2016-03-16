function [CoMDes,qDes,constraints, currentState,impedances,w_H_b,enableIncrement] = ...
    stateMachineWalking(connection,CoM_0, q0, w_CoM, CoMIn, qIn, constraintsIn, wrench_rightFoot,wrench_leftFoot,l_sole_H_b, r_sole_H_b, sm,gain, time, qj)
%#codegen
global state;

global w_H_fixedLink;
global fixedLink;
%     global counterTraj;

%     w_H_fixedLink  = eye(4);
%
%     fixedLink   =  1; % 1 = left, 2 = right

CoMDes      = CoM_0;
constraints = [1; 1];
qDes        = q0;

impedances = gain.impedances(1,:);

% For handling when to switch state
enableIncrement = 1;
qj_left_leg = qj(12:17)';
qj_right_leg = qj(18:23)';

%% STATES
% 1 - initial state (waiting for references)
% 2 - 2 feet CoM Tracking
% 3 - Left foot balancing
% 4 - Right foot balancing

if fixedLink == 1
    w_H_b   =  w_H_fixedLink * l_sole_H_b;
else
    w_H_b   =  w_H_fixedLink * r_sole_H_b;
end

if state == 1
    %waiting for com reference
    CoMDes      = CoM_0;
    
    if norm(CoM_0 - CoMIn) > eps && connection
        state = 2;
    end
    enableIncrement = 1;
elseif state == 2
    
    CoMDes = CoMIn;
    qDes   = qIn;
    impedances  = gain.impedances(state,:);
    
    %         CoMError  = CoMDes - w_CoM;
    CoMError = w_CoM - CoMDes;
    if ~any(constraintsIn - [1; 0]) ...
            && norm(CoMError(2)) < sm.com.threshold
        constraints = [1; 0]; %right foot is no longer a constraint
        fixedLink = 1;
        w_H_fixedLink = w_H_b / l_sole_H_b;
        state = 3;
        
    elseif ~any(constraintsIn - [0; 1]) ...
            && norm(CoMError(2)) < sm.com.threshold
        constraints = [0; 1]; %left foot is no longer a constraint
        state = 4;
        
        fixedLink = 2;
        w_H_fixedLink = w_H_b / r_sole_H_b;
        
    end
    
elseif state == 3
    % Left foot balancing
    constraints = [1; 0];
    
    CoMDes = CoMIn;
    qDes   = qIn;
    impedances  = gain.impedances(state,:);
    
    if wrench_rightFoot(3) > sm.wrench.threshold ...
            && ~any(constraintsIn - [1; 1])
        state = 2;
    end
    
elseif state == 4
    % Right foot balancing
    constraints = [0; 1];
    
    CoMDes = CoMIn;
    qDes   = qIn;
    qDes_right_leg = qDes(18:23)';
    impedances  = gain.impedances(state,:);
    
    if ~any(constraintsIn - [1;1])
        enableIncrement = 0;
        
%         if wrench_leftFoot(3) < sm.wrench.threshold ...
%                 && norm(qj_right_leg - qDes_right_leg) < 1.5 %norma della gamba destra < threshold. Put this threshold in the configuration file for the walking state machine
%             enableIncrement = 1;
%             state = 2;
%         end
        if norm(qj_right_leg - qDes_right_leg) > 0.3
            disp('Increasing integral gain for joint 0');
%             gain.integral(18) = gain.integral(18) + 5;
        end
        if norm(qj_right_leg - qDes_right_leg) < 0.1
            disp('NORM OF THE RIGHT LEG ERROR IS NOW LESS THAN 0.1!!!');
            enableIncrement = 1;
            if wrench_leftFoot(3) < sm.wrench.threshold % "<" because the threshold is negative.
                state = 2;
            end
        end
    end
end

%     counterTraj = counterTraj + 1;
%     disp('From stateMachineWalking.m')
%     disp('Counter Traj: '), disp(counterTraj);
%     disp('at time: '), disp(time);
currentState = state;
