function [constraintMatrixLeftFoot,constraintMatrixRightFoot] = updateConstraintMatrices(ConstraintsMatrix,rotMatLeftFoot,rotMatRightFoot,footSize)
    e1              = [1;0;0];
    e2              = [0;1;0];
    e3              = [0;0;1]; 

    constraintMatrixLeftFoot              = ConstraintsMatrix; 
    constraintMatrixLeftFoot(end-4,1:3)   = -e3'*rotMatLeftFoot;                
    constraintMatrixLeftFoot(end-3,:)     = [ footSize(1,1)*e3'*rotMatLeftFoot', e2'*rotMatLeftFoot'];
    constraintMatrixLeftFoot(end-2,:)     = [-footSize(1,2)*e3'*rotMatLeftFoot',-e2'*rotMatLeftFoot'];

    constraintMatrixLeftFoot(end-1,:)     = [ footSize(2,1)*e3'*rotMatLeftFoot',-e1'*rotMatLeftFoot'];
    constraintMatrixLeftFoot(end  ,:)     = [-footSize(2,2)*e3'*rotMatLeftFoot', e1'*rotMatLeftFoot'];

    constraintMatrixRightFoot              = ConstraintsMatrix;
    constraintMatrixRightFoot(end-4,1:3)   = -e3'*rotMatRightFoot;  
    constraintMatrixRightFoot(end-3,:)     = [ footSize(1,1)*e3'*rotMatRightFoot', e2'*rotMatRightFoot'];
    constraintMatrixRightFoot(end-2,:)     = [-footSize(1,2)*e3'*rotMatRightFoot',-e2'*rotMatRightFoot'];

    constraintMatrixRightFoot(end-1,:)     = [ footSize(2,1)*e3'*rotMatRightFoot',-e1'*rotMatRightFoot'];
    constraintMatrixRightFoot(end  ,:)     = [-footSize(2,2)*e3'*rotMatRightFoot', e1'*rotMatRightFoot'];
end