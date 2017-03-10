function [M_c, h_c, g_c, Jc_c, dJcNu_c, nu_c] = floatingToCentroidalDynamics(M, h, g, Jc, dJcNu, nu, T, dT)
% centroidalConversion 
% converts dynamic equation parameters to the
% corresponding values in centroidal frame of reference

% For reference, see https://traversaro.github.io/preprints/changebase.pdf
% in particular equations 17, 21, 23

%%inputs
% M     : mass matrix computed with respect to base frame
% h     : bias matrix computed with respect to base frame
% g     : gravity forces matrix computed with respect to base frame
% Jc    : Jacobian computed with respect to base frame
% dJcNu : time derivative of Jacobian multiplied by nu, with respect to
%         base frame
% nu    : velocity of the floating base system
% T     : centroidal transformation matrix (transform from base frame to CoM frame)
% dT    : time derivative of T

%%Outputs
% M_c, h_c, g_c : mass, bias and gravity forces matrices with respect to CoM frame
% Jc_c, dJcNu_c : Jc and dJcNu transformed to be in respect to CoM frame
% nu_c          : nu transformed into CoM coordinates

%%
M_c = T' \ M / T;

g_c = T' \ g;

CNu = h - g; %coriolis matrix times nu
C_cNu_c = T' \ CNu - M_c * dT * nu; 
h_c = C_cNu_c + g_c;

nu_c = T * nu;

Jc_c = Jc / T;

dJcNu_c = dJcNu - Jc / T * dT  * nu;


% % % Alternative computations, for sanity check:
% % % ndof  = size(g,1)-6;
% % % invT  = eye(ndof+6)/T;
% % % invdT = - invT * dT * invT;
% % % 
% % % M_c = invT' * M * invT;
% % % 
% % % g_c = invT' * g;
% % % 
% % % CNu = h - g;
% % % C_cNu_c = invT' * (M * invdT * T * Nu + CNu);
% % % h_c = C_cNu_c + g_c;
% % % 
% % % Nu_c = T * Nu;
% % % 
% % % Jc_c = Jc * invT;
% % % 
% % % dJcNu_c = dJcNu + (Jc * invdT) * (T * Nu);

end