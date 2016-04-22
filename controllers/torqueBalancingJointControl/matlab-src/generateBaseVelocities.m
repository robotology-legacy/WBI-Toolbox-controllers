function Nu_base = generateBaseVelocities(JcLeftFoot,JcRightFoot,Constraints,dq,reg)

Jc      = [Constraints(1)*JcLeftFoot;
           Constraints(2)*JcRightFoot];

pinvJb  = (Jc(:,1:6)'*Jc(:,1:6) + reg.pinvDampVb*eye(6))\Jc(:,1:6)';  
  
Nu_base = -pinvJb*Jc(:,7:end)*dq;

end
