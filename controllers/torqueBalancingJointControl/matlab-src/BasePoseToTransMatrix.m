function qRef_ikin = BasePoseToTransMatrix(qRef_ikin_no_converted)

% define the base pose in quaternions
qBase = qRef_ikin_no_converted(1:7);

% define the rotation matrix
[xBase,RBase] = frame2posrot(qBase);

% generate the transformation matrix
TransMatrix  = [RBase xBase; [0 0 0 1]];

% column-major serialization
vectorizedTrans  = TransMatrix(:);

% new state vector
qRef_ikin    = [vectorizedTrans; qRef_ikin_no_converted(8:end)];

end