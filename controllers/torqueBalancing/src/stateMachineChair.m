function [w_H_b,constraints,impedances,kpCom,kdCom,currentState,jointsSmoothingTime] = ...
          stateMachineChair(r_sole_H_b,l_sole_H_b,r_leg_H_b,l_leg_H_b,t,gain,sm)
      
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) 
        state         = sm.stateAt0;
        tSwitch       = 0;
        w_H_fixedLink = eye(4);
    end

    constraints = [1; 1];
    w_H_b       = eye(4);
    impedances  = gain.impedances(1,:);
    kpCom       = gain.PCOM(1,:);   
    kdCom       = gain.DCOM(1,:);   

    %% BALANCING ON THE LEGS
    if state == 1 
        w_H_b      =  w_H_fixedLink * l_leg_H_b;

        if t > sm.tBalancing % after tBalancing time start moving CoM forward
           
            state = 2;           
        end
    end

    %% COM TRANSITION
    if state == 2 
        
        
    end

    %% LOOKING FOR CONTACT
    if state == 3 
        
        
    end
    
    %% TWO FEET BALANCING
    if state == 4 
       
    end

currentState        = state;
jointsSmoothingTime = sm.jointsSmoothingTimes(state);
    
end
 
   