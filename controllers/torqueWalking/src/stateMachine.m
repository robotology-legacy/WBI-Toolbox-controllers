function [desired_x_dx_ddx_root, desired_R_w_dw_root, desired_s_ds_dds] = stateMachine(s0, w_H_root_0, root_displacement, ROBOT_DOF)

% Keep initial root position + a desired displacement
desired_x_root = w_H_root_0(1:3, 4) + root_displacement;
desired_x_dx_ddx_root = [desired_x_root, zeros(3,2)];

% Keep initial root orientation
desired_R_w_dw_root = [w_H_root_0(1:3,1:3), zeros(3,2)];

% Keep initial joint positions
desired_s_ds_dds = [s0, zeros(ROBOT_DOF,2)];

end

