function I_desRotAcc = rotationalPID(rotMatrix,I_angVel,I_desRotMatAngVelAcc,gainsPD)
    
    % rotMatrix = I_R_B: rotation matrix transforming a vector expressed w.r.t. a body frame B, e.g. p_B, into 
    %                    a vector expressed in the inertial frame I, e.g. p_A, i.e.
    %            
    %                                   p_I = I_R_B * p_B
    %
    % I_angVel  = omega_I: angular velocity expressed w.r.t. the inertial frame I, i.e.
    %
    %                                   dot(I_R_B) = S(I_angVel) * I_R_B
    
    B_angVel      = rotMatrix'*I_angVel;
    desRotMat     = I_desRotMatAngVelAcc(1:3,1:3);
    B_desAngVel   = desRotMat'*I_desRotMatAngVelAcc(:,4);
    
    B_desRotAcc   = I_desRotMatAngVelAcc(:,5) ...
                    - gainsPD(:,1)*gainsPD(:,2)*invSkew(desRotMat'*rotMatrix)  ...
                    - gainsPD(:,2)*(B_angVel-B_desAngVel)...
                    - gainsPD(:,1)*invSkew(desRotMat'*rotMatrix*skew(B_angVel) - skew(B_desAngVel)*desRotMat'*rotMatrix);
    
    I_desRotAcc   = rotMatrix*B_desRotAcc;
end
