function fixedLink_R_root  = rootRotation(l_sole_H_root,r_sole_H_root,state)

    persistent w_H_fixedLink;
    persistent previousState;
    
    
    if isempty(w_H_fixedLink) 
        w_H_fixedLink = eye(4);
        previousState = 1;
    end
            
   
    if state == 7 && previousState == 6
        w_H_fixedLink        = w_H_fixedLink*l_sole_H_root/r_sole_H_root;
    elseif state == 1 && previousState == 11
        w_H_fixedLink        = eye(4); %w_H_fixedLink*r_sole_H_root/l_sole_H_root;
    end

    if state <= 6 % left foot balancing
        fixedLink_H_root     = w_H_fixedLink * l_sole_H_root;
   
    else %if state > 6 && state <= 11  % right foot balancing
        fixedLink_H_root     = w_H_fixedLink * r_sole_H_root;   
    end
    
    fixedLink_R_root = fixedLink_H_root(1:3,1:3);
    previousState = state;
    
end