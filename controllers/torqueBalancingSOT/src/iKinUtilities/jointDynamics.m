function jointAccelerations = jointDynamics(jointAngles, desJointAngles, jointVelocity, gain)

jointAccelerations = -gain.impedances * (jointAngles-desJointAngles)...
                     -gain.dampings   * jointVelocity;
                          
end