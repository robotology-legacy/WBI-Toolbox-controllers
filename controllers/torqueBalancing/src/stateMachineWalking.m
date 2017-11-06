function     [w_H_b, CoMDes, qDes, constraints, impedances, kpCom, kdCom, currentState, jointsSmoothingTime] = ...
              stateMachineWalking(wrench_rightFoot, wrench_leftFoot, CoM_0, q0, x_CoM, qj, t, sm, gain,...
                                  CoMDesInput, qjDesInput, constraintsInput, w_H_bInput)
                         
    % codegen
    persistent state;
    persistent tSwitch;

    if isempty(state) || isempty(tSwitch) 
        state         = sm.stateAt0;
        tSwitch       = 0;
    end
    
    constraints = [1; 1];
    impedances  = gain.impedances(1,:);
    kpCom       = gain.PCOM(1,:);   
    kdCom       = gain.DCOM(1,:);  
    CoMError    = CoMDesInput(1:3) - x_CoM;

    %% TWO FEET BALANCING
    if state == 1
        
        % nothing to do actually. Just wait to switch in another state
        
        if t > sm.tBalancing && ~sm.demoOnlyBalancing && wrench_rightFoot(3) < sm.wrench.thresholdContactOff && norm(CoMError(2)) < sm.com.threshold
            
           state = 2; % switch to left foot balancing
           
        elseif t > sm.tBalancing && ~sm.demoOnlyBalancing && wrench_leftFoot(3) < sm.wrench.thresholdContactOff && norm(CoMError(2)) < sm.com.threshold
           
           state = 3; % switch to right foot balancing
        end
    end

    %% LEFT FOOT BALANCING 
    if state == 2
 
        constraints = [1; 0]; % right foot is no longer a constraints

        % update gains
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if t > sm.tBalancing && ~sm.demoOnlyBalancing && wrench_rightFoot(3) < sm.wrench.thresholdContactOn && norm(CoMError(2)) < sm.com.threshold
            
           state = 1; % go back to two feet balancing
        end
    end
    
    %% RIGHT FOOT BALANCING 
    if state == 3
 
        constraints = [0; 1]; % left foot is no longer a constraints

        % update gains
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if t > sm.tBalancing && ~sm.demoOnlyBalancing && wrench_leftFoot(3) < sm.wrench.thresholdContactOn && norm(CoMError(2)) < sm.com.threshold
            
           state = 1; % go back to two feet balancing
        end
    end
 
    %% The following variables actually does not depend upon the current state
    CoMDes      = CoMDesInput;
    qDes        = qjDesInput;
    w_H_b       = w_H_bInput;
    
    %% Current state and smoothing time
    currentState        = state;
    jointsSmoothingTime = sm.jointsSmoothingTimes(state);
    
end
