function [LfootPos_error,LfootOri_error,RfootPos_error,RfootOri_error,CoMPos] = positionAndOrientationErrors...
         (fwdkin_CoM,fwdkin_lfoot,fwdkin_lfootInit,fwdkin_rfoot,fwdkin_rfootInit)
%% Position errors
lFootPos         = fwdkin_lfoot(1:3,4);
rFootPos         = fwdkin_rfoot(1:3,4);
lFootPos_init    = fwdkin_lfootInit(1:3,4);
rFootPos_init    = fwdkin_rfootInit(1:3,4);
CoMPos           = fwdkin_CoM(1:3,4);
LfootPos_error   = lFootPos-lFootPos_init;
RfootPos_error   = rFootPos-rFootPos_init;

%% Rotation matrices at feet
R_lfoot          = fwdkin_lfoot(1:3,1:3);
R_rfoot          = fwdkin_rfoot(1:3,1:3);
R_lfoot_init     = fwdkin_lfootInit(1:3,1:3);
R_rfoot_init     = fwdkin_rfootInit(1:3,1:3);

% feet orientation is parametrized with Euler angles
[~,lFootOri]          = fromRotMatr_toZYXEulerAngles(R_lfoot);
[~,rFootOri]          = fromRotMatr_toZYXEulerAngles(R_rfoot);
[~,rFootOri_init]     = fromRotMatr_toZYXEulerAngles(R_rfoot_init);
[~,lFootOri_init]     = fromRotMatr_toZYXEulerAngles(R_lfoot_init);

%% Feet orientation errors
LfootOri_error        = lFootOri-lFootOri_init;
RfootOri_error        = rFootOri-rFootOri_init;

end