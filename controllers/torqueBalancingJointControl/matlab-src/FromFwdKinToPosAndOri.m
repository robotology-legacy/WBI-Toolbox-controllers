function [delta_lfootPos,delta_lfootOri,delta_rfootPos,delta_rfootOri,CoMPos] = FromFwdKinToPosAndOri(fwdkinCoM,fwdkin_lfoot,fwdkin_lfootIni,...
                                                                                                      fwdkin_rfoot,fwdkin_rfootIni)                                                                                                                                                                                                  
%% Positions
lFootPos        = fwdkin_lfoot(1:3,4);
rFootPos        = fwdkin_rfoot(1:3,4);
CoMPos          = fwdkinCoM(1:3,4);
lFootPos_ini    = fwdkin_lfootIni(1:3,4);
rFootPos_ini    = fwdkin_rfootIni(1:3,4);

delta_lfootPos  = lFootPos-lFootPos_ini;
delta_rfootPos  = rFootPos-rFootPos_ini;

%% Rotation matrices at the feet
R_b_lfoot     = fwdkin_lfoot(1:3,1:3);
R_b_rfoot     = fwdkin_rfoot(1:3,1:3);

R_b_lfoot_ini = fwdkin_lfootIni(1:3,1:3);
R_b_rfoot_ini = fwdkin_rfootIni(1:3,1:3);

% orientation is parametrized with euler angles
[~,lFootOri]          = parametrization(R_b_lfoot);
[~,rFootOri]          = parametrization(R_b_rfoot);

[~,rFootOri_ini]      = parametrization(R_b_rfoot_ini);
[~,lFootOri_ini]      = parametrization(R_b_lfoot_ini);
  
delta_lfootOri        = lFootOri-lFootOri_ini;
delta_rfootOri        = rFootOri-rFootOri_ini;

end