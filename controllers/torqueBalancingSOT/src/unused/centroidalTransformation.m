function [T, dT] = centroidalTransformation(xcom,x_b,dxcom,dx_b,M)
% centroidalTransformation
% converts the normal floating base frame of reference to 
% the centroidal frame of reference


%%inputs
% xcom  : position of origin of CoM frame with respect to base frame
% x_b   : position of origin of base frame with respect to base frame
% dxcom : velocity of origin of CoM frame with respect to base frame
% dx_b  : velocity of origin of base frame with respect to base frame
% M     : mass matrix. Can be seen as a structure of the following form
%           M = [ b_I_c  Fb;
%                 Fb'    Hb ]

%%Outputs
% T     : centroidal transformation matrix (transform from base frame to CoM frame)
%         For reference, see https://traversaro.github.io/preprints/changebase.pdf
%         section 5.3 is about centroidal change of variable

%dT     : time derivative of T -- no references for this one


%% T calculation
r       = xcom - x_b;

g_X_b   = [eye(3),   skew(r)';
           zeros(3), eye(3)  ];
      
b_I_c   = M(1:6, 1:6);
Fb      = M(1:6, 7:end);
ndof    = size(Fb,2);

S_gb    = g_X_b * (b_I_c \ Fb);

T       = [g_X_b,         S_gb;
           zeros(ndof,6), eye(ndof)];
       
                 
%% time derivative of T
dr      = dxcom - dx_b;
 
dg_X_b  = [zeros(3),skew(dr)';
           zeros(3),zeros(3)];

mdr     = M(1,1)*dr;       
db_I_c  = [zeros(3),  skew(mdr)';
           skew(mdr), zeros(3) ];
       
inv_db_I_C = -b_I_c\db_I_c/b_I_c;

dS_gb   = dg_X_b*(b_I_c\Fb) + g_X_b*inv_db_I_C*Fb;
 
dT      = [dg_X_b,        dS_gb;
           zeros(ndof,6), zeros(ndof)];

end