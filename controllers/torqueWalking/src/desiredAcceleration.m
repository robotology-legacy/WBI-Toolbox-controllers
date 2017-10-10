function [dnu_des, root_x_error, root_w_error, s_error] ...
          = desiredAcceleration(s, ds, w_H_root, rootVelocity, ...
                                desired_x_dx_ddx_root, desired_R_w_dw_root, desired_s_ds_dds, ...
                                gain)

%Root linear acceleration
x_root        = w_H_root(1:3, 4);
dx_root       = rootVelocity(1:3);
ddx_star = linearPID(x_root, dx_root, desired_x_dx_ddx_root, [gain.x_root.p, gain.x_root.d]);

%Root angular acceleration
w_R_root     = w_H_root(1:3,1:3);
w_root       = rootVelocity(4:6);
dw_star     = rotationalPID(w_R_root,w_root,desired_R_w_dw_root, [gain.w_root.p, gain.w_root.d]);

%Joint accelerations
dds_star    = linearPID(s, ds, desired_s_ds_dds, [gain.joints.p, gain.joints.d]);

%Desired action feedback
dnu_des    = [ddx_star; dw_star; dds_star];


%Debug information
root_x_error = x_root - desired_x_dx_ddx_root(:,1);
root_w_error = invSkew(desired_R_w_dw_root(1:3,1:3)' * w_R_root);
s_error     = s - desired_s_ds_dds(:,1);

end

