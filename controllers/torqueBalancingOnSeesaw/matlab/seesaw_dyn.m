function [acc, F_c] = seesaw_dyn(w_R_s, omega_w, wrench_ext, seesaw, v_w, w_p_s)
%%input:
%%
%% seesaw: structure
%%  - R: radius of the enclosing circle
%%  - delta: distance between the center of the circle and the seesaw com.
%%  - inertia: inertia matrix w.r.t. the center of mass frame
%%  - mass: mass of the seesaw

e3 = [0; 0; 1];
r_w = -seesaw.R * e3 + seesaw.delta * w_R_s * e3;

g_w = [0; 0; -9.81];
s_R_w = w_R_s';

g_s = s_R_w * g_w;
r_s  = s_R_w * r_w;
omega_s = s_R_w * omega_w;
s_dR_w = -S(omega_s) * s_R_w;
% dr_s = s_dR_w * r_w - s_R_w * seesaw.delta * s_dR_w * e3;
dr_s = seesaw.R * S(omega_s) * s_R_w * e3;
v_s = s_R_w * v_w;

v_p = v_s + S(omega_s) * r_s;
p_p = w_p_s + r_w;
p_p = s_R_w * p_p;
Kp = 0* 10 * eye(3);
Kv = 2 * sqrt(Kp);


F_ext = s_R_w * wrench_ext(1:3);
mu_ext = s_R_w * wrench_ext(4:6);

inertia_inv = inv(seesaw.inertia);

force_matrix = S(r_s) * inertia_inv * S(r_s) - eye(3)/seesaw.mass;
force_rhs = g_s + S(omega_s)^2 * r_s + F_ext/seesaw.mass - ...
	 	S(dr_s) * omega_s + S(r_s) * inertia_inv * (S(omega_s) * seesaw.inertia * omega_s - mu_ext) + ...
        Kv * v_p + Kp * p_p;

F_c = force_matrix \ force_rhs;

acc = zeros(6, 1);
acc(4:6) = inertia_inv * (S(r_s) * F_c  + mu_ext - S(omega_s) * seesaw.inertia * omega_s);
acc(1:3) = S(dr_s) * omega_s + S(r_s) * acc(4:6);

lin_acc = g_s - S(omega_s) * v_s + F_c / seesaw.mass + F_ext / seesaw.mass;
% lin_acc - acc(1:3)
%
% any(lin_acc - acc(1:3))

% acc(1:3) = lin_acc;
acc(4:6) = w_R_s * acc(4:6);
acc(1:3) = w_R_s * (acc(1:3) + S(omega_s) * v_s);


end
