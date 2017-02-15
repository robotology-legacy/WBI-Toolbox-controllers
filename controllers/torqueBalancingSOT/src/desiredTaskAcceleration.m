function [desiredTaskAcc,comError,rootRotErr,lFootPosErr,lFootRotErr,rFootPosErr,rFootRotErr, taskAccErr] = ...
              desiredTaskAcceleration(comPos,desComPosVelAcc,...
                                      w_R_root,desRootRotPosVelAcc,...
                                      w_H_l_sole,desLFootRotPosVelAcc,desLFootOriginPosVelAcc,...
                                      w_H_r_sole,desRFootRotPosVelAcc,desRFootOriginPosVelAcc,...
                                      taskVelocities,gainsPDCom,gain)
                                              
    comVel             = taskVelocities( 1: 3);
    rootAngVel         = taskVelocities( 4: 6);
    
    leftFootLinVel     = taskVelocities( 7: 9);
    leftFootAngVel     = taskVelocities(10:12);
    
    rightFootLinVel    = taskVelocities(13:15);
    rightFootAngVel    = taskVelocities(16:18);
    
    comDDotStar        = linearPID(comPos,comVel,desComPosVelAcc,gainsPDCom);
    rootOmegaDotStar   = rotationalPID(w_R_root,rootAngVel,desRootRotPosVelAcc,gain.rootPD);  
    
    lFootPosDDotStar   = linearPID(w_H_l_sole(1:3,4),leftFootLinVel,desLFootOriginPosVelAcc,gain.lFoot.posPD);
    lFootOmegaDotStar  = rotationalPID(w_H_l_sole(1:3,1:3),leftFootAngVel,desLFootRotPosVelAcc,gain.lFoot.rotPD);  
    
    rFootPosDDotStar   = linearPID(w_H_r_sole(1:3,4),rightFootLinVel,desRFootOriginPosVelAcc,gain.rFoot.posPD);
    rFootOmegaDotStar  = rotationalPID(w_H_r_sole(1:3,1:3),rightFootAngVel,desRFootRotPosVelAcc,gain.rFoot.rotPD);
    
    desiredTaskAcc     = [comDDotStar;
                          rootOmegaDotStar
                          lFootPosDDotStar
                          lFootOmegaDotStar
                          rFootPosDDotStar
                          rFootOmegaDotStar];
    
    comError          = comPos - desComPosVelAcc(:,1);
    rootRotErr        = invSkew(desRootRotPosVelAcc(1:3,1:3)'*w_R_root);
    lFootPosErr       = w_H_l_sole(1:3,4) - desLFootOriginPosVelAcc(:,1);
    lFootRotErr       = invSkew(desLFootRotPosVelAcc(1:3,1:3)'*w_H_l_sole(1:3,1:3));
    rFootPosErr       = w_H_r_sole(1:3,4) - desRFootOriginPosVelAcc(:,1);
    rFootRotErr       = invSkew(desRFootRotPosVelAcc(1:3,1:3)'*w_H_r_sole(1:3,1:3));
    
    taskAccErr = [comError;
                  rootRotErr;
                  lFootPosErr;
                  lFootRotErr;
                  rFootPosErr;
                  rFootRotErr];
        
end