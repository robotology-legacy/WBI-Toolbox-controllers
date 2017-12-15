function w_H_fixedLink = fromQuatToTransfMatrix(qt_fixed_LFoot, LFoot_is_active, qt_fixed_RFoot)


    q    = qt_fixed_RFoot;
    toll = 0.1;
    
    if LFoot_is_active > (1-toll)
        
        q = qt_fixed_LFoot;
    end
    
    % transformation matrix from position + quaternions rapresentation
    w_R_fixedLink = rotationFromQuaternion(q(4:7));
    
    w_H_fixedLink = [w_R_fixedLink, q(1:3);
                       0   0   0    1];
    
end