function [desired_x_dx_ddx_CoM, desired_Intw_w_dw_CoM, desired_q_dq_ddq] = stateMachine(q0, w_H_CoM_0, ROBOT_DOF)

% Keep initial CoM position
desired_x_dx_ddx_CoM = [w_H_CoM_0(1:3, 4), zeros(3,2)];

% Keep initial CoM orientation
desired_Intw_w_dw_CoM = [w_H_CoM_0(1:3,1:3), zeros(3,2)];

% Keep initial joint positions
desired_q_dq_ddq = [q0, zeros(ROBOT_DOF,2)];

end

