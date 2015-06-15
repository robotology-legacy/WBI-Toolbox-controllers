function dx = robot_and_seesaw_dyn(~, x, u, model)
    %% Dynamics of the robot + seesaw.
    %% The two dynamics are highly coupled and must be computed together.
    %% Indeed to fully compute the dynamics of the seesaw, the external wrenches must be known.
    %% These wrenches are (in part) due to the robot. But in order to compute the wrenches 
    %% the robot is doing on the seesaw, the angular velocity of the latter must be known.
    %%
    %% Input (everything expressed w.r.t. inertial frame)
    %% - t: current time (not needed)
    %% - x: current state (R(), q, nu, omega) \in \mathbb{R}^{SE(3)+n + 6+n + SE(3) + 3}
    %% - u: current control 
    %% - data: struct with fields:
    %%   - seesaw: struct with seesaw model
    %%   - robot: struct with robot model - NO (we use internally the mex-wbi for now)
    %%
    %% Output:
    %% dx: the dynamic function (dx = f(x, u))
    %%

    e3 = [0; 0; 1];
    g_w = [0; 0; -9.81];
    
    %state is integrated in the "world frame"
    robotDoFs      = model.robot.dofs;
    [ss_pos, ss_vel, ~, robot_vel] = state_partitioning(x, robotDoFs);
    seesaw_quat    = ss_pos(4:7);
    omega_w        = ss_vel(4:6);
    
    w_R_s          = rotationFromQuaternion(seesaw_quat);
    
    %% seesaw part
    seesaw         = model.seesaw;

    e3             = [0;0;1];

    r_w            = seesaw.delta * w_R_s * e3 - seesaw.R * e3 ; %vector between com and contact point in world frame
    s_R_w          = w_R_s';
    g_s            = s_R_w * g_w; %gravity in the seesaw frame
    r_s            = s_R_w * r_w; %r_w in seesaw frame
    omega_s        = s_R_w * omega_w; %omega in seesaw frame
    dr_s           = seesaw.R * S(omega_s) * s_R_w * e3; %derivative of r_s

    s_s_l          = [0; model.robot.lFootCentreDistance; model.seesaw.top];
    s_s_r          = [0; model.robot.rFootCentreDistance; model.seesaw.top];

    w_s_l          = w_R_s * s_s_l;
    w_s_r          = w_R_s * s_s_r;

    
    As             = [ eye(3), zeros(3), eye(3), zeros(3); ...
                       S(w_s_l), eye(3), S(w_s_r), eye(3)];
          
    Theta          = eye(3) + S(r_s) * seesaw.iota* S(r_s)';
    Iota_r         = eye(3) + seesaw.iota*norm(r_s)^2;

    Omega_0        = seesaw.iota + (1/det(Theta))*seesaw.iota*S(r_s)*Iota_r*S(r_s)*seesaw.iota;

    Omega_1        = (1/det(Theta))*seesaw.iota*Theta*S(r_s)*(S(dr_s) * omega_s - S(omega_s)^2 * r_s - g_s);
    DOmega_1       = -Omega_0*S(omega_s)*seesaw.invIota*omega_s;
    Omega_1        = Omega_1 + DOmega_1 ;

    Omega_2        = [-seesaw.iota*Theta*S(r_s)/det(Theta),Omega_0]*1/seesaw.mass;
    
    %%Robot
    JdotNu         = zeros(12, 1);
    J              = zeros(12, robotDoFs + 6);
    JdotNu(1:6)    = wbm_djdq('l_sole');
    JdotNu(7:end)  = wbm_djdq('r_sole');
    J(1:6,:)       = wbm_jacobian('l_sole');
    J(7:end,:)     = wbm_jacobian('r_sole');
    Mass           = wbm_massMatrix();
    genBiasForces  = wbm_generalisedBiasForces();
    
    
    Delta          = [S(r_s - s_s_l); eye(3); S(r_s - s_s_r); eye(3)];
    DeltaDot       = [eye(3); zeros(3); eye(3); zeros(3)] * S(dr_s);
    
    s_feetVels     =  Delta * omega_s;
    
    robotBias      = J / Mass * (genBiasForces - [zeros(6,1); u]) ...
                     - JdotNu ...
                     + blkdiag(w_R_s,w_R_s,w_R_s,w_R_s) * ( ...
                     blkdiag(S(omega_s),S(omega_s),S(omega_s),S(omega_s)) * s_feetVels ...
                     + DeltaDot * omega_s + Delta * Omega_1...
                     );
    
    
    feet_forces    = (blkdiag(w_R_s, w_R_s, w_R_s, w_R_s)*Delta* Omega_2* blkdiag(s_R_w,s_R_w) * As + J / Mass * J'  )\robotBias;
                             
    w_omega_dot = w_R_s * (Omega_1 - Omega_2*blkdiag(w_R_s',w_R_s')*As*feet_forces);
    
    w_nu_dot = Mass \ ([zeros(6, 1); u] + J' * feet_forces - genBiasForces);
    
    lin_acc_ss = S(dr_s) * omega_s + S(r_s) * w_omega_dot;

    
    dx = [w_R_s * (lin_acc_ss + S(omega_s) * s_R_w * ss_vel(1:3));
          w_omega_dot;
          w_nu_dot];
        
end