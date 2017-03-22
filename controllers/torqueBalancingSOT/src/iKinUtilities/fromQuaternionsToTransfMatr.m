function qDes = fromQuaternionsToTransfMatr(qDesQuat)
% define the base pose in quaternions
qBase = qDesQuat(1:7);

% define the rotation matrix
[xBase,Rbase] = frame2posrot(qBase);

% generate the transformation matrix
transfMatrix  = [Rbase xBase; [0 0 0 1]];

% column-major serialization
vectorizedTrans  = transfMatrix(:);

% new state vector
qDes    = [vectorizedTrans; qDesQuat(8:end)];

end