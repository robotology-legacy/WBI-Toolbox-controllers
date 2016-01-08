function [CoMDes,qDes,constraints, currentState,impedances,w_H_b] = ...
    stateMachineWalking(CoM_0, q0, l_sole_CoM,r_sole_CoM,qj, t, wrench_rightFoot,wrench_leftFoot,l_sole_H_b, r_sole_H_b, sm,gain)
    %#codegen
    global state;
    global tSwitch;
    global w_H_r_sole_switch;
    
    global w_H_fixedLink;
    global fixedLink;
    
    w_H_fixedLink  = eye(4);
    fixedLink   =  1; % 1 = left, 2 = right
    
    CoMDes      = CoM_0;
    constraints = [1; 1];
    qDes        = q0;

    impedances = gain.impedances(1,:);
    
    
    %% STATES
    % 1 - initial state (waiting for references)
    % 2 - 2 feet CoM Tracking
    % 3 - Left foot balancing
    % 4 - Right foot balancing
    
    if fixedLink == 1
        w_H_b   =  w_H_fixedLink * l_sole_H_b;
    else 
        w_H_b   =  w_H_fixedLink * r_sole_H_b;
    end
    
    if state == 1
        %waiting for com reference
        if abs(CoM_0 - CoMIn) < eps % abs(CoM_0 - CoMDes) < eps
            state = 2;
        end
    elseif state == 2
        
        CoMDes = CoMIn;
        qDes   = qIn;
        
        
        
        CoMError  = CoMDes - l_sole_CoM;
        
        if constraintsIn == [1; 0] && norm(CoMError(2)) < sm.com.threshold
            constraints = [1; 0]; %right foot is no longer a constraints
            fixedLink = 1;
            w_H_fixedLink = w_H_b / l_sole_H_b;
            state = 3;
            
        elseif constraintsIn == [0; 1] && norm(CoMError(2)) < sm.com.threshold
            constraints = [0; 1]; %left foot is no longer a constraints
            state = 4;
            
            fixedLink = 2;
            w_H_fixedLink = w_H_b / r_sole_H_b;
           
        end
            
        
        
    elseif state == 3
        % Left foot balancing
        constraints = [1; 0];
        
        CoMDes = CoMIn;
        qDes   = qIn;
        
        
        %w_H_b   =  w_H_fixedLink * l_sole_H_b;
        
        if wrench_rightFoot(3) > sm.wrench.threshold
            state = 2;
        end
        
    elseif state == 4
        % Right foot balancing
        constraints = [0; 1];
        
        CoMDes = CoMIn;
        qDes   = qIn;
        
        %w_H_b   =  w_H_fixedLink * r_sole_H_b;
        
        if wrench_leftFoot(3) > sm.wrench.threshold
            state = 2;
        end
    end
            
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    %% Two feet balancing.
    if state == 1 
        if t > sm.tBalancing %after tBalancing time start moving weight to the left
           state = 2; 
        end
    end

    %% Left transition
    if state == 2 
%         CoMDes(2)    =  sm.com.states(state,2)'; %new reference for CoM
        
        CoMDes(1:2) = sm.com.states(state,1:2)';    
        
        CoMError  = CoMDes - l_sole_CoM;
        qDes      = sm.joints.states(state,:)'; % new reference for q
        
        if norm(CoMError(2)) < sm.com.threshold
           state = 3; 
           tSwitch = t;
        end
    end

    %% Left foot balancing
    if state == 3 
        constraints = [1; 0]; %right foot is no longer a constraints
%         constraints = [0; 1]; %left foot is no longer a constraints

        CoMDes(2)   = sm.com.states(state,2)'; %new reference for CoM
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if t > tSwitch + sm.DT % yoga
            state   = 4;
            tSwitch = t;
        end
    end
    
    %% YOGA LEFT FOOT
    if state == 4 
        constraints = [1; 0]; %right foot is no longer a constraints
%         constraints = [0; 1]; %left foot is no longer a constraints

        CoMDes(2)  =  sm.com.states(state,2)'; %new reference for CoM
        qDes       =  sm.joints.states(state,:)';
        impedances = gain.impedances(state,:);

        for i = 1: size(sm.joints.pointsL,1)-1
            if t > (sm.joints.pointsL(i,1) + tSwitch) && t <= (sm.joints.pointsL(i+1,1)+ tSwitch)
                qDes = sm.joints.pointsL(i,2:end)';
            end
        end
        if t > sm.joints.pointsL(end,1) + tSwitch 
            qDes = sm.joints.pointsL(end,2:end)';
            if  (t > (sm.joints.pointsL(end,1) + tSwitch + sm.joints.smoothingTime+sm.joints.pauseTimeLastPostureL))
                state   = 5;
                tSwitch = t;
            end
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 5 
        constraints = [1; 0]; %right foot is no longer a constraints
%         constraints = [0; 1]; %left foot is no longer a constraints

        CoMDes(2)    =  sm.com.states(state,2)'; %new reference for CoM
        qDes        =  sm.joints.states(state,:)';
        impedances = gain.impedances(state,:);

        qTileRightLeg = qj(end-5:end)-qDes(end-5:end);
        
        qTileLeftLeg = qj(end-11:end-6)-qDes(end-11:end-6);
            
        if norm(qTileRightLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTileLeftLeg)*180/pi < sm.joints.thresholdInContact
            state   = 6;
            tSwitch = t;
        end
    end
    %% LOOKING FOR A CONTACT
    if state == 6 
        constraints = [1; 0]; %right foot is no longer a constraints
%         constraints = [0; 1]; %left foot is no longer a constraints

        CoMDes(2)   = sm.com.states(state,2)'; %new reference for CoM
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if wrench_rightFoot(3) > sm.wrench.threshold
            state = 7;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 7 
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances = gain.impedances(state,:);
        if ((norm(l_sole_CoM(1:2)-CoMDes(1:2)) < 2*sm.com.threshold) && sm.yogaAlsoOnRightFoot && (t > tSwitch + sm.tBalancing))
            w_H_r_sole_switch   = l_sole_H_b/r_sole_H_b;
            state               = 8;
            tSwitch             = t;
        end
    end
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
     
%% TRANSITION TO THE RIGHT FOOT
    if state == 8 
        constraints = [1; 1]; %right foot is no longer a constraints
        w_H_b   =  w_H_r_sole_switch*r_sole_H_b;

        impedances = gain.impedances(state,:);
        qDes       =  sm.joints.states(state,:)';
        
             
        CoMDes(1:2) = w_H_r_sole_switch(1:2,4) + sm.com.states(state,1:2)';    

        w_CoM = w_H_r_sole_switch*[r_sole_CoM;1];
        
        CoMError  = CoMDes - w_CoM(1:3);
        
        if norm(CoMError(2)) < sm.com.threshold
           state = 9; 
           tSwitch = t;
        end

    end
    
    
     %% RIGHT FOOT BALANCING 
    if state == 9
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b   =  w_H_r_sole_switch*r_sole_H_b;

        CoMDes(1:2) = w_H_r_sole_switch(1:2,4) + sm.com.states(state,1:2)';    
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if t > tSwitch + sm.DT % yoga
            state   = 10;
            tSwitch = t;
        end
    end
    
    %% YOGA RIGHT FOOT
    if state == 10 
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b   =  w_H_r_sole_switch*r_sole_H_b;

        CoMDes(1:2) = w_H_r_sole_switch(1:2,4) + sm.com.states(state,1:2)';    
        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        for i = 1: size(sm.joints.pointsR,1)-1
            if t > (sm.joints.pointsR(i,1) + tSwitch) && t <= (sm.joints.pointsR(i+1,1)+ tSwitch)
                qDes = sm.joints.pointsR(i,2:end)';
            end
        end
        if t > sm.joints.pointsR(end,1) + tSwitch 
            qDes = sm.joints.pointsR(end,2:end)';
            if  (t > sm.joints.pointsR(end,1) + tSwitch + sm.joints.smoothingTime + sm.joints.pauseTimeLastPostureR ) 
                state   = 11;
                tSwitch = t;
            end
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 11 
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b   =  w_H_r_sole_switch*r_sole_H_b;

        CoMDes(1:2) = w_H_r_sole_switch(1:2,4) + sm.com.states(state,1:2)';    
        qDes        =  sm.joints.states(state,:)';
        impedances = gain.impedances(state,:);

        qTileRightLeg = qj(end-5:end)-qDes(end-5:end);
        
        qTileLeftLeg = qj(end-11:end-6)-qDes(end-11:end-6);
            
%             norm(qTileRightLeg)*180/pi
        if norm(qTileRightLeg)*180/pi < sm.joints.thresholdInContact && norm(qTileLeftLeg)*180/pi < sm.joints.thresholdNotInContact
            state   = 12;
            tSwitch = t;
        end
    end
    %% LOOKING FOR A CONTACT
    if state == 12
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b   =  w_H_r_sole_switch*r_sole_H_b;

%         CoMDes(1:2) = w_H_r_sole_switch(1:2,4) + sm.com.states(state,1:2)';    
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if wrench_leftFoot(3) > sm.wrench.threshold
            state   = 13;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 13
        w_H_b   =  w_H_r_sole_switch*r_sole_H_b;
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances = gain.impedances(state,:);
        if t > sm.tBalancing %after tBalancing time start moving weight to the left
%            state = 2; 
        end
    end 
    
    currentState = state;
