function inverseKinRef = generateJointRef(dNuRef_ikin, NuRef_ikin, qRef_ikin)

% composing the joints references 
ddqjRef = dNuRef_ikin(7:end);
dqjRef  = NuRef_ikin(7:end);
qjRef   = qRef_ikin(8:end);

inverseKinRef = [qjRef dqjRef ddqjRef];

end