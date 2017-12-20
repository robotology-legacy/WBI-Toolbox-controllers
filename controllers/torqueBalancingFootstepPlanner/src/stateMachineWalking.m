function [CoMDes, impedances, kpCom, kdCom, currentState, constraints] = ...
             stateMachineWalking(wrench_rightFoot, wrench_leftFoot, CoM_0, q0, x_CoM, qj, t, sm, gain,...
                                 CoMDesInput, qjDesInput, constraintsInput)
                         
    % state machine for teleoperated walking
    persistent state;
    persistent constraintsState;

    if isempty(state) 
        state         = sm.stateAt0;
    end
    
    if isempty(constraintsState) 
        constraintsState = sm.constraintsAt0;
    end
    
    toll        = 0.1;
    
    %% TWO FEET BALANCING
    if constraintsInput(1)>(1-toll) && constraintsInput(2)>(1-toll) && ...
        (wrench_rightFoot(3) > sm.wrench.thresholdContactOn) && ...
        (wrench_leftFoot(3) > sm.wrench.thresholdContactOn)
        
        constraintsState = [1; 1];
        state = 1;
    end

    %% LEFT FOOT BALANCING 
    if constraintsInput(1)>(1-toll) && constraintsInput(2)<(toll)
        
        constraintsState = [1; 0];
        state       = 2;
    end
    
    %% RIGHT FOOT BALANCING 
    if constraintsInput(1)<(toll) && constraintsInput(2)>(1-toll)
  
        constraintsState = [0; 1];
        state       = 3;    
    end
 
    %% The following variables actually do not depend upon the current state
    CoMDes       = [CoMDesInput(1:3),CoMDesInput(4:6),CoMDesInput(7:9)];
    
    %% Current state and smoothing time
    currentState = state;
    constraints = constraintsState;
    impedances  = gain.impedances(state,:);
    kpCom       = gain.PCOM(state,:);   
    kdCom       = gain.DCOM(state,:); 
      
end
