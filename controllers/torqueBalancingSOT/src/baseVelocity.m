function y = baseVelocity(JcLeftFoot, JcRightFoot, feetConstraints, dqj, reg)

Jc     = [feetConstraints(1) * JcLeftFoot;
          feetConstraints(2) * JcRightFoot];

pinvJb = pinvDamped(Jc(:,1:6), reg.pinvDampVb);  
  
y      = -pinvJb * Jc(:,7:end) * dqj;