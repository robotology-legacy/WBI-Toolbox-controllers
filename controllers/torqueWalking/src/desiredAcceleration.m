function [dnu_des, ddfeet_star, root_x_error, root_w_error, s_error] ...
          = desiredAcceleration(s, ds, w_H_root, w_H_l_sole, w_H_r_sole,...
                                rootVelocity, l_soleVelocity, r_soleVelocity, ...
                           desired_x_dx_ddx_root, desired_R_w_dw_root, desired_x_dx_ddx_l_sole, desired_x_dx_ddx_r_sole, desired_s_ds_dds, ...
                                gain)

%Root linear acceleration
x_root        = w_H_root(1:3, 4);
dx_root       = rootVelocity(1:3);
ddx_star = linearPID(x_root, dx_root, desired_x_dx_ddx_root, [gain.x_root.p, gain.x_root.d]);

%Root angular acceleration
w_R_root     = w_H_root(1:3,1:3);
w_root       = rootVelocity(4:6);
dw_star     = rotationalPID(w_R_root,w_root,desired_R_w_dw_root, [gain.w_root.p, gain.w_root.d]);

%Feet linear and angular acceleration
x_l_sole       = w_H_l_sole(1:3, 4);
x_r_sole       = w_H_r_sole(1:3, 4);
w_R_l_sole     = w_H_l_sole(1:3,1:3);
w_R_r_sole     = w_H_r_sole(1:3,1:3);

dx_l_sole      = l_soleVelocity(1:3);
dx_r_sole      = r_soleVelocity(1:3);
w_l_sole       = l_soleVelocity(4:6);
w_r_sole       = r_soleVelocity(4:6);

desired_x_posVelAcc_l_sole = [desired_x_dx_ddx_l_sole(1:3, 4), desired_x_dx_ddx_l_sole(1:3, 5:6)];
desired_w_posVelAcc_l_sole = [desired_x_dx_ddx_l_sole(1:3, 1:3), desired_x_dx_ddx_l_sole(1:3, 5:6)];
desired_x_posVelAcc_r_sole = [desired_x_dx_ddx_r_sole(1:3, 4), desired_x_dx_ddx_r_sole(1:3, 5:6)];
desired_w_posVelAcc_r_sole = [desired_x_dx_ddx_r_sole(1:3, 1:3), desired_x_dx_ddx_r_sole(1:3, 5:6)];

ddxl_sole_star = [linearPID(x_l_sole, dx_l_sole,    desired_x_posVelAcc_l_sole, [gain.x_root.p, gain.x_root.d]);
                  rotationalPID(w_R_l_sole,w_l_sole,desired_w_posVelAcc_l_sole, [gain.w_root.p, gain.w_root.d])];
ddxr_sole_star = [linearPID(x_r_sole, dx_r_sole,    desired_x_posVelAcc_r_sole, [gain.x_root.p, gain.x_root.d]);
                  rotationalPID(w_R_r_sole,w_r_sole,desired_w_posVelAcc_r_sole, [gain.w_root.p, gain.w_root.d])];
ddfeet_star = [ddxl_sole_star; ddxr_sole_star];

%Joint accelerations
dds_star    = linearPID(s, ds, desired_s_ds_dds, [gain.joints.p, gain.joints.d]);

%Desired action feedback
dnu_des    = [ddx_star; dw_star; dds_star];


%Debug information
root_x_error = x_root - desired_x_dx_ddx_root(:,1);
root_w_error = invSkew(desired_R_w_dw_root(1:3,1:3)' * w_R_root);
s_error     = s - desired_s_ds_dds(:,1);

end

