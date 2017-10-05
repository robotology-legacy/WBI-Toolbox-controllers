function w_v_base  = computeBaseVelocity(J_l_sole, w_H_l_sole,dq_j, s_omega_s, model)
%#codegen
    s_omega_s      = s_omega_s * (pi/180);

    w_R_s          = w_H_l_sole(1:3,1:3);

    w_omega_s      = w_R_s*s_omega_s;

    e3 = [0;0;1];
    
    w_r            = model.seesaw.delta * w_R_s * e3 - model.seesaw.rho * e3 ; 

    s_s_l          = [ 0; 
                       model.seesaw.lFootDistanceCenter; 
                       model.seesaw.top];

    w_s_l          = w_R_s * s_s_l;

    w_v_l_sole     = [Sf(w_r - w_s_l)*w_omega_s; 
                                      w_omega_s  ];

    w_v_base       = J_l_sole(1:6,1:6)\(w_v_l_sole - J_l_sole(1:6,7:end)*dq_j);
end