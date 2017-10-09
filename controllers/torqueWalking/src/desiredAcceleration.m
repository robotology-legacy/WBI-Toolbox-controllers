function [dnu_star, CoM_x_error, CoM_w_error, s_error] ...
          = desiredAcceleration(s, ds, w_H_CoM, CoMVelocity, ...
                                desired_x_dx_ddx_CoM, desired_R_w_dw_CoM, desired_s_ds_dds, ...
                                gain)

%CoM linear acceleration
xCoM        = w_H_CoM(1:3, 4);
dxCoM       = CoMVelocity(1:3);
ddx_star = linearPID(xCoM, dxCoM, desired_x_dx_ddx_CoM, [gain.x_CoM.p, gain.x_CoM.d]);

%CoM angular acceleration
w_R_CoM     = w_H_CoM(1:3,1:3);
w_CoM       = CoMVelocity(4:6);
dw_star     = rotationalPID(w_R_CoM,w_CoM,desired_R_w_dw_CoM, [gain.w_CoM.p, gain.w_CoM.d]);

%Joint accelerations
dds_star    = linearPID(s, ds, desired_s_ds_dds, [gain.joints.p, gain.joints.d]);

%Desired action feedback
dnu_star    = [ddx_star; dw_star; dds_star];


%Debug information
CoM_x_error = xCoM - desired_x_dx_ddx_CoM(:,1);
CoM_w_error = invSkew(desired_R_w_dw_CoM(1:3,1:3)' * w_R_CoM);
s_error     = s - desired_s_ds_dds(:,1);

end

