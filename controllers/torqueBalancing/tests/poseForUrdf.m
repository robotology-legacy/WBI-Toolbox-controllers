%% Generates the relative pose of the contact frames w.r.t. the upper legs frames.
%% It is then used in the .urdf file

Rup_H_Rcont =[0.0468    0.0002    0.9989   -0.0596;
             -0.9764    0.2111    0.0457   -0.0694;
             -0.2108   -0.9775    0.0100   -0.0008;
                   0         0         0    1.0000];

Lup_H_Lcont =[0.0466   -0.0000    0.9989   -0.0596;
             -0.9757   -0.2142    0.0455   -0.0700;
              0.2140   -0.9768   -0.0100   -0.0009;
                   0         0         0    1.0000];
               
%% Rotations and positions               
Rup_R_Rcont = Rup_H_Rcont(1:3,1:3);
Lup_R_Lcont = Lup_H_Lcont(1:3,1:3);
Rup_pos_Rc  = Rup_H_Rcont(1:3,4);
Lup_pos_Lc  = Lup_H_Lcont(1:3,4);

disp(Rup_pos_Rc)
disp(Lup_pos_Lc)

% Right leg conversion to iDynTree
Rup_R_Rcont_id = iDynTree.Rotation();
Rup_R_Rcont_id.fromMatlab(Rup_R_Rcont);

Rup_pos_Rc_id = iDynTree.Position();
Rup_pos_Rc_id.fromMatlab(Rup_pos_Rc);

% Left leg conversion to iDynTree
Lup_R_Lcont_id = iDynTree.Rotation();
Lup_R_Lcont_id.fromMatlab(Lup_R_Lcont);

Lup_pos_Lc_id = iDynTree.Position();
Lup_pos_Lc_id.fromMatlab(Lup_pos_Lc);

%% iDynTree transformation matrices
iDynTreeTransform_L = iDynTree.Transform(Lup_R_Lcont_id,Lup_pos_Lc_id);
iDynTreeTransform_R = iDynTree.Transform(Rup_R_Rcont_id,Rup_pos_Rc_id);

rpy_l = iDynTreeTransform_L.getRotation().asRPY();
rpy_l.toMatlab()

rpy_r = iDynTreeTransform_R.getRotation().asRPY();
rpy_r.toMatlab()

