function [feetDynamics,CoMDynamics,jointsDynamics]  = feetCoMJointDynamics(LfootPos_error,RfootPos_error,RfootOri_error,LfootOri_error,...
                                                                           xCoM,qj,desired_x_dx_ddx_CoM,desired_qj_dqj_ddqj,LfootVel,RfootVel,...
                                                                           dxCoM,dqj,gain)
%% Feet error dynamics
feetDynamics    = [(-gain.ikin.Kpfeet*LfootPos_error(1:3)-gain.ikin.Kdfeet*LfootVel(1:3));
                   (-gain.ikin.Kpfeet*transpose(LfootOri_error(1:3))-gain.ikin.Kdfeet*LfootVel(4:6));
                   (-gain.ikin.Kpfeet*RfootPos_error(1:3)-gain.ikin.Kdfeet*RfootVel(1:3));
                   (-gain.ikin.Kpfeet*transpose(RfootOri_error(1:3))-gain.ikin.Kdfeet*RfootVel(4:6))];
               
%% CoM error dynamics                                         
CoMDynamics     = desired_x_dx_ddx_CoM(:,3)-gain.ikin.KpCoM*(xCoM-desired_x_dx_ddx_CoM(:,1))-gain.ikin.KdCoM*(dxCoM-desired_x_dx_ddx_CoM(:,2));

%% Joint error dynamics
jointsDynamics  = desired_qj_dqj_ddqj(:,3)-gain.ikin.impedances*(qj-desired_qj_dqj_ddqj(:,1))-gain.ikin.dampings*(dqj-desired_qj_dqj_ddqj(:,2));

end


% % function [feetDynamics,CoMDynamics,jointsDynamics]  = feetCoMJointDynamics(LfootPos_error,RfootPos_error,RfootOri_error,LfootOri_error,...
% %                                                                            xCoM,qj,desired_x_dx_ddx_CoM,desired_qj_dqj_ddqj,LfootVel,RfootVel,...
% %                                                                            dxCoM,dqj,gain)
% % %% Feet error dynamics
% % feetDynamics    = [(-gain.ikin.Kpfeet*LfootPos_error(1:3)-gain.ikin.Kdfeet*LfootVel(1:3));
% %                    (-gain.ikin.Kpfeet*transpose(LfootOri_error(1:3))-gain.ikin.Kdfeet*LfootVel(4:6));
% %                    (-gain.ikin.Kpfeet*RfootPos_error(1:3)-gain.ikin.Kdfeet*RfootVel(1:3));
% %                    (-gain.ikin.Kpfeet*transpose(RfootOri_error(1:3))-gain.ikin.Kdfeet*RfootVel(4:6))];
% %                
% % %% CoM error dynamics                                         
% % CoMDynamics     = desired_x_dx_ddx_CoM(:,3)-gain.ikin.KpCoM*(xCoM-desired_x_dx_ddx_CoM(:,1))-gain.ikin.KdCoM*(dxCoM-desired_x_dx_ddx_CoM(:,2));
% % %here, add dynamics on rotation too!
% % % rootOriDynamics  = rotationalPID(w_R_root,rootAngVel,desRootRotPosVelAcc,gain.rootPD);
% % 
% % %% Joint error dynamics
% % jointsDynamics  = desired_qj_dqj_ddqj(:,3)-gain.ikin.impedances*(qj-desired_qj_dqj_ddqj(:,1))-gain.ikin.dampings*(dqj-desired_qj_dqj_ddqj(:,2));
% % 
% % end