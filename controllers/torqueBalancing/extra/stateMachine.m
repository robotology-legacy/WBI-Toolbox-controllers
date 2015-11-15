function [CoMDes,qDes,constraints, currentState,impedances] = stateMachine(CoM_0, q0, CoM,qj, t, wrench_right, sm,gain)
    %#codegen
    global state;
    global tSwitch;
    
    CoMDes      = CoM_0;
    constraints = [1; 1];
    qDes        = q0;

    impedances = gain.impedances(1,:);
    
    %% Two feet balancing.
    if state == 1 
        if t > sm.tBalancing %after tBalancing time start moving weight to the left
           state = 2; 
        end
    end

    %% Left transition
    if state == 2 
        CoMDes(2)    =  sm.com.states(state,2)'; %new reference for CoM
        CoMError  = CoMDes - CoM;
        qDes      = sm.joints.states(state,:)'; % new reference for q
        if norm(CoMError(2)) < sm.com.threshold
           state = 3; 
           tSwitch = t;
        end
    end

    %% Left foot balancing
    if state == 3 
        constraints = [1; 0]; %right foot is no longer a constraints
        CoMDes(2)   = sm.com.states(state,2)'; %new reference for CoM
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if t > tSwitch + sm.DT % yoga
            state = 4;
            tSwitch = t;
        end
    end
    
    %% YOGA LEFT FOOT
    if state == 4 
        constraints = [1; 0]; %right foot is no longer a constraints
        CoMDes(2)    =  sm.com.states(state,2)'; %new reference for CoM
        qDes        =  sm.joints.states(state,:)';
        impedances = gain.impedances(state,:);

        for i = 1: size(sm.joints.points,1)-1
            if t > (sm.joints.points(i,1) + tSwitch) && t <= (sm.joints.points(i+1,1)+ tSwitch)
                qDes = sm.joints.points(i,2:end)';
            end
        end
        if t > sm.joints.points(end,1) + tSwitch 
            qDes = sm.joints.points(end,2:end)';
            impedances  = gain.impedances(state+1,:);
            
            qTileRightLeg = qj(end-5:end)-qDes(end-5:end);
            
%             norm(qTileRightLeg)*180/pi
            if norm(qTileRightLeg)*180/pi < sm.joints.threshold && (t > sm.joints.points(end,1) + tSwitch + sm.joints.smoothingTime) 
                state   = 5;
                tSwitch = t;
            end
        end
    end
    
    if state == 5 
        constraints = [1; 0]; %right foot is no longer a constraints
        CoMDes(2)   = sm.com.states(state,2)'; %new reference for CoM
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if wrench_right(3) > sm.wrench.threshold
            state = 6;
            tSwitch = t;
        end
    end

    if state == 6 
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances = gain.impedances(state,:);
        if t > tSwitch + sm.DT
            CoMDes(2)    =  sm.com.states(state,2)'; %new reference for CoM
        end
    end
    
    currentState = state;
    
    % if state == 3
    %     stateVec = [0;0;1;0];
    %     qDes(end-11:end)      =  qDesRightFoot(2,end-11:end);
    %     CoMDes_t     = CoMDes(2,:)';
    % %     CoMError  = CoMDes_t - CoM;
    %     if  abs(wR(3)) > forceThreshold  %norm(CoMError) < CoMErrorThreshold && 
    %        state = 4; 
    %        tSwitch = t;
    %     end
    % end
    % 
    % if state == 4
    %   	stateVec = [1;0;0;0];
    %     qDes(end-11:end)      =  qDesRightFoot(2,end-11:end);
    %     CoMDes_t     = CoMDes(2,:)';
    % end