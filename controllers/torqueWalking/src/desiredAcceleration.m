function dnu_star = desiredAcceleration(qj, dqj, w_H_CoM, CoMVelocity, ...
                           desired_x_dx_ddx_CoM, desired_Intw_w_dw_CoM, desired_q_dq_ddq, ...
                           gain)

%CoM linear acceleration
xCoM        = w_H_CoM(1:3, 4);
dxCoM       = CoMVelocity(1:3);
ddot_x_star = linearPID(xCoM, dxCoM, desired_x_dx_ddx_CoM, [gain.x_CoM.p, gain.x_CoM.d]);

%CoM angular acceleration
w_R_CoM     = w_H_CoM(1:3,1:3);
w_CoM       = CoMVelocity(4:6);
dw_star     = rotationalPID(w_R_CoM,w_CoM,desired_Intw_w_dw_CoM, [gain.w_CoM.p, gain.w_CoM.d]);

%Joint accelerations
dqq_star    = linearPID(qj, dqj, desired_q_dq_ddq, [gain.joints.p, gain.joints.d]);

%Desired action feedback
dnu_star    = [ddot_x_star; dw_star; dqq_star];

end

