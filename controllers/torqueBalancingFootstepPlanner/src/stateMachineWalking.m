function [CoMDes, impedances, kpCom, kdCom, currentState] = ...
             stateMachineWalking(wrench_rightFoot, wrench_leftFoot, CoM_0, q0, x_CoM, qj, t, sm, gain,...
                                 CoMDesInput, qjDesInput, constraintsInput)
                         
    % state machine for teleoperated walking
    persistent state;

    if isempty(state) 
        state         = sm.stateAt0;
    end
    
    impedances  = gain.impedances(1,:);
    kpCom       = gain.PCOM(1,:);   
    kdCom       = gain.DCOM(1,:); 
    toll        = 0.1;
    
    %% TWO FEET BALANCING
    if constraintsInput(1)>(1-toll) && constraintsInput(2)>(1-toll)
        
        % nothing to do actually. Just wait to switch in another state
        state = 1;
    end

    %% LEFT FOOT BALANCING 
    if constraintsInput(1)>(1-toll) && constraintsInput(2)<(toll)

        % update gains
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:); 
        state       = 2;
    end
    
    %% RIGHT FOOT BALANCING 
    if constraintsInput(1)<(toll) && constraintsInput(2)>(1-toll)
 
        % update gains
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        state       = 3;
    end
 
    %% The following variables actually does not depend upon the current state
    CoMDes       = [CoMDesInput(1:3),CoMDesInput(4:6),CoMDesInput(7:9)];
    
    %% Current state and smoothing time
    currentState = state;
      
end
