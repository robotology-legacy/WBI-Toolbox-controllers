function I_desRotAcc = rotationalPID(rotMatrix,I_angVel,I_desRotMatAngVelAcc,gainsPD)
    
    B_angVel      = rotMatrix'*I_angVel;
    desRotMat     = I_desRotMatAngVelAcc(1:3,1:3);
    B_desAngVel   = desRotMat'*I_desRotMatAngVelAcc(:,4);
    
    B_desRotAcc   = I_desRotMatAngVelAcc(:,5) ...
                    - gainsPD(:,1)*gainsPD(:,2)*invSkew(desRotMat'*rotMatrix)  ...
                    - gainsPD(:,2)*(B_angVel-B_desAngVel)...
                    - gainsPD(:,1)*invSkew(desRotMat'*rotMatrix*skew(B_angVel) - skew(B_desAngVel)*desRotMat'*rotMatrix);
    
    I_desRotAcc   = rotMatrix*B_desRotAcc;
end