function baseDer = baseDerivative(nuDes_base, basePose)

% apply the conversion of reference frame between base and wolrd
omegaBase      = nuDes_base(4:end);
[~,Rbase]      = frame2posrot(basePose);
omegaWorld     = transpose(Rbase)*omegaBase;

% calculate the quaternion derivative
quat_base      = basePose(4:end);
dquat_base     = quaternionDerivative(quat_base,omegaWorld,1);

% calculate the base pose derivative
baseDer        = [nuDes_base(1:3);dquat_base];
end