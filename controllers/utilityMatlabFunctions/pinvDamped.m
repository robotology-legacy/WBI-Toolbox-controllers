function pinvDampA = pinvDamped(A,reg)
    pinvDampA = A'/(A*A' + reg*eye(size(A,1)));
end