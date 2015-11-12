function [CoMDes,qDes,constraints] = stateMachine(CoM_0, q0, CoM, t, references)
    %#codegen
    global state;
    global tSwitch;

    CoMDes      = CoM_0;
    constraints = [1;1];
    qDes        = q0;

    %% Two feet balancing.
    if state == 0 
        if t > references.tBalancing %after tBalancing time start moving weight to the left
           state = 1; 
        end
    end

    %% Left transition
    if state == 1 
        CoMDes    =  references.com.states(1,:)'; %new reference for CoM
        CoMError  = CoMDes - CoM;
        qDes      = references.joints.states(1,:)'; % new reference for q
        if norm(CoMError) < references.com.threshold
           state = 2; 
           tSwitch = t;
        end
    end

    %% Left foot balancing
    if state == 2 
        constraints = [1; 0]; %right foot is no longer a constraints
        CoMDes      =  references.com.states(1,:)';
        qDes        =  references.joints.states(1,:)';

        if t > tSwitch + references.DT % yoga
            for i = 1: size(references.joints.points,1)-1
                if t > references.joints.points(i,1) && t <= references.joints.points(i+1,1)
                    qDes = references.joints.points(i,2:end)';
                end
            end
        end
    end

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