function w_H_b_updated = baseToWorldTransformation(poseLeftFoot,poseRightFoot,w_H_lr,w_H_b,constraints)
%#codegen

    w_H_l         = homTranformFromAxisAngle(poseLeftFoot);
    
    w_H_r         = homTranformFromAxisAngle(poseRightFoot);
    
    l_H_b         = w_H_l\w_H_b;

    r_H_b         = w_H_r\w_H_b;

    w_H_b_updated = w_H_lr*(constraints(1)*l_H_b + constraints(2)*(1-constraints(1))*r_H_b );
    
%     w_H_b
%     
%     w_H_l
%     
%     w_H_lr
%     
%     constraints
%     
%     w_H_b_updated
    
end