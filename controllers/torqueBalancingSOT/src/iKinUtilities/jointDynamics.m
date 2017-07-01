function jointAccelerations = jointDynamics(jointAngles, desJointAngles, jointVelocity, impedances, dampings)

jointAccelerations = -diag(impedances) * (jointAngles-desJointAngles)...
                     -diag(dampings)   * jointVelocity;
                          
end