function desiredTaskAcc = desiredTaskAcceleration(comPos,desComPosVelAcc,...
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
end