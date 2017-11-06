function [Mc, hc, Jcc, dJcDq_c, nu_c] = centroidalTransform(Jtasks, dJnu_tasks, nu, w_H_b, M, h, g, taskPos, CONFIG)

if CONFIG.QP.USE_CENTROIDAL_TRANSFORMATION
    %Perform centroidal transformation
    
    CoMPos = taskPos(1:3, 4);
    CoMVel = [Jtasks(1:3,:) * nu; zeros(3,1)];
    
    [T, dT] = centroidalTransformationT_TDot(CoMPos, w_H_b(1:3,4), CoMVel, nu(1:6), M);
    
    [Mc, C_cNu_c, gc, Jcc, dJcDq_c, nu_c] = fromFloatingToCentroidalDynamics(M, h, g, Jtasks, dJnu_tasks, nu, T, dT);
    
    hc = C_cNu_c + gc;
    
else %do not perform any change
    
    Mc = M;
    hc = h;
    Jcc = Jtasks;
    dJcDq_c = dJnu_tasks;
    nu_c = nu;

end

