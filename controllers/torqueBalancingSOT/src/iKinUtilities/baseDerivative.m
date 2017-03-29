function baseDer = baseDerivative(nuDes_base, basePose)

% apply the conversion of reference frame between base and wolrd
omegaBase      = nuDes_base(4:end);
quat_base      = basePose(4:end);
Rbase          = rotationFromQuaternion(quat_base);
omegaWorld     = transpose(Rbase)*omegaBase;

% calculate the quaternion derivative
dquat_base     = quaternionDerivative(quat_base,omegaWorld,1);

% calculate the base pose derivative
baseDer        = [nuDes_base(1:3);dquat_base];
end