function [desired_x_dx_ddx_CoM, desired_Intw_w_dw_CoM, desired_s_ds_dds] = stateMachine(s0, w_H_CoM_0, ROBOT_DOF)

% Keep initial CoM position + a desired displacement
CoM_displacement = [0; -0.5; 0];

% Move to the left
desired_x_CoM = w_H_CoM_0(1:3, 4) + CoM_displacement;
desired_x_dx_ddx_CoM = [desired_x_CoM, zeros(3,2)];

% Keep initial CoM orientation
desired_Intw_w_dw_CoM = [w_H_CoM_0(1:3,1:3), zeros(3,2)];

% Keep initial joint positions
desired_s_ds_dds = [s0, zeros(ROBOT_DOF,2)];

end

