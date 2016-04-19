function [CoMDes,qDes,constraints, currentState,impedances,w_H_b,jointsSmoothingTime] = ...
    stateMachine(CoM_0, q0, l_sole_CoM,r_sole_CoM,qj, t, ...
                  wrench_rightFoot,wrench_leftFoot,l_sole_H_b, r_sole_H_b, sm,gain)
    %#codegen
    global state;
    global tSwitch;
    global w_H_r_sole_switch;
    
    w_H_b       = l_sole_H_b;
    CoMDes      = CoM_0;
    constraints = [1; 1];
    qDes        = q0;

    impedances = gain.impedances(1,:);
        
    %% Two feet balancing.
    if state == 1 
        if t > sm.tBalancing %after tBalancing time start moving weight to the left
           state = 2;
           if sm.demoOnlyRightFoot
                w_H_r_sole_switch   = l_sole_H_b/r_sole_H_b;
                state = 8;
           end
           
        end
    end

    %% TRANSITION TO THE LEFT FOOT
    if state == 2 
%         CoMDes(2)    =  sm.com.states(state,2)'; %new reference for CoM
        
        CoMDes(1:2) = sm.com.states(state,1:2)';    
        impedances = gain.impedances(state,:);

        CoMError  = CoMDes - l_sole_CoM;
        qDes      = sm.joints.states(state,:)'; % new reference for q
        
        if norm(CoMError(2)) < sm.com.threshold && wrench_rightFoot(3) < sm.wrench.thresholdContactOff
           state = 3; 
           tSwitch = t;
        end
    end

    %% LEFT FOOT BALANCING 
    if state == 3 
        constraints = [1; 0]; %right foot is no longer a constraints
%         constraints = [0; 1]; %left foot is no longer a constraints

        CoMDes(2)   = sm.com.states(state,2)'; %new reference for CoM
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);

        if t > tSwitch + sm.DT % yoga
            state   = 4;
            tSwitch = t;
            if sm.jumpYoga
                state   = 5;
            end
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
            if  (t > (sm.joints.pointsL(end,1) + tSwitch + sm.jointsSmoothingTimes(state)+sm.joints.pauseTimeLastPostureL))
                state   = 5;
                tSwitch = t;
            end
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 5 
        constraints = [1; 0]; %right foot is no longer a constraints
%         constraints = [0; 1]; %left foot is no longer a constraints

        CoMDes(2)  =  sm.com.states(state,2)'; %new reference for CoM
        qDes       =  sm.joints.states(state,:)';
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

        if wrench_rightFoot(3) > sm.wrench.thresholdContactOn
            state = 7;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 7 
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances = gain.impedances(state,:);
        if ((norm(l_sole_CoM(1:2)-CoMDes(1:2)) < 10*sm.com.threshold) && sm.yogaAlsoOnRightFoot && (t > tSwitch + sm.tBalancing))
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

        if norm(CoMError(2)) < sm.com.threshold  && wrench_leftFoot(3) < sm.wrench.thresholdContactOff
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
            if sm.jumpYoga
                state   = 11;
            end
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
            if  (t > sm.joints.pointsR(end,1) + tSwitch + sm.jointsSmoothingTimes(state) + sm.joints.pauseTimeLastPostureR ) 
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

        if wrench_leftFoot(3) > sm.wrench.thresholdContactOn 
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
           if sm.yogaInLoop
              state = 2; 
              if sm.demoOnlyRightFoot
                 state = 8;
              end
           end
        end
    end 
    
    currentState        = state;
    jointsSmoothingTime = sm.jointsSmoothingTimes(state);
