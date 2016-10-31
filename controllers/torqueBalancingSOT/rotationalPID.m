function desRotAcc = rotationalPID(rotMatrix,angVel,desRotMatAngVelAcc,gainsPD)
    
    desRotMat = desRotMatAngVelAcc(1:3,1:3);
    desAngVel = desRotMatAngVelAcc(:,4);
    
    desRotAcc = desRotMatAngVelAcc(:,5) ...
                - gainsPD(:,1)*gainsPD(:,2)*invSkew(desRotMat'*rotMatrix)  ...
                - gainsPD(:,2)*(angVel-desAngVel)...
                - gainsPD(:,1)*invSkew(desRotMat'*rotMatrix*skew(angVel) - skew(desAngVel)*desRotMat'*rotMatrix);
end