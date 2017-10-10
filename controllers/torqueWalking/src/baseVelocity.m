function nu  = baseVelocity(joint_velocities, w_J_l_sole, w_J_r_sole, feetActivation, reg)

Jc     = [w_J_l_sole * feetActivation(1);
          w_J_r_sole * feetActivation(2)];

pinvJb = pinvDamped(Jc(:,1:6), reg.pinvDamp);  
  
base_velocity      = -pinvJb * Jc(:,7:end) * joint_velocities;

nu = [base_velocity; joint_velocities];


end

