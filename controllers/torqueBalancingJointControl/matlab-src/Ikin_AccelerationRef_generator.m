function dotNu_ikin   = Ikin_AccelerationRef_generator(JFeet, JCoM, JPosture, dJFeetNu, dJCoMNu,iKinFeetCorr, iKinComCorr, iKinPostCorr)

%setup parameters
n_joints  = length(JCoM(1,7:end));
PINV_TOL  = 5e-7;  

% null space projectors for primary and secondary task
NullFeet   = eye(6+n_joints) - pinv(JFeet,PINV_TOL)*JFeet;
NullCoM    = eye(6+n_joints) - pinv(JCoM*NullFeet,PINV_TOL)*JCoM*NullFeet;

%reference acceleration calculation: primary task: feet position and orientation
dotNu_feet   = pinv(JFeet,PINV_TOL)*(iKinFeetCorr - dJFeetNu);

%secondary task: CoM dynamics
dotNu_com    =  pinv(JCoM*NullFeet, PINV_TOL)*(iKinComCorr - dJCoMNu - JCoM*dotNu_feet);

%third task: posture
dotNu_post   = pinv(JPosture*NullFeet*NullCoM, PINV_TOL)*(iKinPostCorr - JPosture*dotNu_feet -JPosture*NullFeet*dotNu_com);

dotNu_ikin   = dotNu_feet + NullFeet*(dotNu_com + NullCoM*dotNu_post);

end