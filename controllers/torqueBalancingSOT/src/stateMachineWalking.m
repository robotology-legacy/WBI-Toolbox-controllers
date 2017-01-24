function [w_H_b, CoMDes,qDes,constraints,impedances,kpCom,kdCom,currentState,jointsSmoothingTime,...
          desRootRotPosVelAcc,desLFootRotPosVelAcc,desLFootOrigin,desRFootRotPosVelAcc,desRFootOrigin] = ...
     stateMachineWalking(CoM_0, q0, l_sole_CoM,r_sole_CoM,qj, t, ...
                  wrench_rightFoot,wrench_leftFoot,l_sole_H_b, r_sole_H_b, sm,gain)
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent w_H_r_sole0;
    persistent l_sole_H_b0;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) || isempty(w_H_r_sole0)  || isempty(l_sole_H_b0) 
        state            = sm.stateAt0;
        tSwitch          = 0;
        w_H_fixedLink    = eye(4);
        w_H_r_sole0      = l_sole_H_b/r_sole_H_b;
        l_sole_H_b0      = l_sole_H_b;
    end
    
    CoMDes      = CoM_0;
    constraints = [1; 1];
    qDes        = q0;
    w_H_b       = eye(4);
    impedances  = gain.impedances(1,:);
    kpCom       = gain.PCOM(1,:);   
    kdCom       = gain.DCOM(1,:);   

    desRootRotPosVelAcc     = [l_sole_H_b0(1:3,1:3),zeros(3,2)];
    desLFootRotPosVelAcc    = [eye(3),zeros(3,2)];
    desLFootOrigin          = zeros(3,1);
    desRFootRotPosVelAcc    = [eye(3),zeros(3,2)];
    desRFootOrigin          = w_H_r_sole0(1:3,4);
    
    %% STATES
    % 1 - Two feet balancing
    % 2 - Transition to left foot
    % 3 - Left foot balancing
    % 4 - Preparing for switching
    % 5 - Looking for contact
    % 6 - Transition to initial position
    % 7 - Transition to right foot
    % 8 - Right foot balancing
    % 9 - Preparing for switching
    %10 - Looking for contact
    %11 - Transition to initial position
 
    %% 1 - TWO FEET BALANCING
    if state == 1 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
        
        impedances = gain.impedances(state,:);
        kpCom      = gain.PCOM(state,:);   
        kdCom      = gain.DCOM(state,:); 
        
        CoMDes     = CoM_0;   
        qDes       = q0;

        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        CoMError   = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
        
        if norm(CoMError(2)) < sm.com.threshold && t > (tSwitch + sm.tBalancing) %after tBalancing time start next state
           state = 2;
           if sm.demoOnlyRightFoot
                w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
                state = 7;
           end
           
        end
    end
    
    %% 2 - TRANSITION TO THE LEFT FOOT
    if state == 2 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;      
        
        impedances = gain.impedances(state,:);
        kpCom      = gain.PCOM(state,:);   
        kdCom      = gain.DCOM(state,:);
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes     = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';   
        qDes       = sm.joints.states(state,:)';

        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        CoMError   = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
       
        if norm(CoMError(2)) < sm.com.threshold && wrench_rightFoot(3) < sm.wrench.thresholdContactOff
           state = 3; 
           tSwitch = t;
        end
    end
    
    %% 3 - LEFT FOOT BALANCING 
    if state == 3 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        constraints = [1; 0]; %right foot is no longer a constraint

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
        desRFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        
        if t > tSwitch + sm.DT 
            state   = 4;
            tSwitch = t;
        end
    end
    
    
    %% 4 - PREPARING FOR SWITCHING
    if state == 4 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        constraints = [1; 0]; %right foot is no longer a constraint        

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:); 
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';
  
        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);        
        qTildeLLeg  = qj(end-11:end-6)-qDes(end-11:end-6);        

        if norm(qTildeRLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdInContact
            state   = 5;
            tSwitch = t;
        end
    end
    
    %% 5 - LOOKING FOR A CONTACT
    if state == 5
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        constraints = [1; 0]; %right foot is no longer a constraint

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        = sm.joints.states(state,:)';

        if wrench_rightFoot(3) > sm.wrench.thresholdContactOn
            state = 6;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 6 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        constraints = [1; 1];
        
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);
        
        CoMDes     = CoM_0;   
        qDes       = q0;
        
        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        CoMError   = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);
        
        if norm(CoMError(2)) < sm.com.threshold && norm(qTildeRLeg)*180/pi < sm.joints.thresholdNotInContact && t > (tSwitch + sm.tBalancing)
            state           = 7;
            tSwitch         = t;
            w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
        end
    end
    
    %% TRANSITION TO THE RIGHT FOOT
    if state == 7 
        w_H_b       =  w_H_fixedLink * r_sole_H_b;
        constraints = [1; 1];

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);  
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';
         
        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];       
        CoMError    = fixed_link_CoMDes(1:3) - r_sole_CoM(1:3);

        if norm(CoMError(2)) < sm.com.threshold  && wrench_leftFoot(3) < sm.wrench.thresholdContactOff
           state = 8; 
           tSwitch = t;
        end

    end
    
    
        %% 8 - LEFT FOOT BALANCING 
    if state == 8 
        w_H_b       =  w_H_fixedLink * r_sole_H_b;
        constraints = [0; 1]; %left foot is no longer a constraint

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
        desLFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';
        
        if t > tSwitch + sm.DT 
            state   = 9;
            tSwitch = t;
        end
    end
    
    
    %% 9 - PREPARING FOR SWITCHING
    if state == 9 
        w_H_b       =  w_H_fixedLink * r_sole_H_b;
        constraints = [0; 1]; %left foot is no longer a constraint        

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:); 
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';
  
        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);        
        qTildeLLeg  = qj(end-11:end-6)-qDes(end-11:end-6);        

        if norm(qTildeLLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeRLeg)*180/pi < sm.joints.thresholdInContact
            state   = 10;
            tSwitch = t;
        end
    end
    
    
    %% 10 - LOOKING FOR A CONTACT
    if state == 10
        w_H_b       =  w_H_fixedLink * r_sole_H_b;
        constraints = [0; 1]; %left foot is no longer a constraint

        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        = sm.joints.states(state,:)';

        if wrench_leftFoot(3) > sm.wrench.thresholdContactOn
            state = 11;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 11 
        w_H_b       =  w_H_fixedLink * r_sole_H_b;
        constraints = [1; 1];
        
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);
        
        CoMDes     = CoM_0;   
        qDes       = q0;
        qTildeLLeg  = qj(end-5:end)-qDes(end-5:end);
        
        if norm(qTildeLLeg)*180/pi < sm.joints.thresholdNotInContact && t > (tSwitch + sm.tBalancing)
            if sm.yogaInLoop
              state = 1;
              tSwitch = t;
              w_H_fixedLink   = w_H_fixedLink*r_sole_H_b/l_sole_H_b;
              if sm.demoOnlyRightFoot
                 state = 7;           
                 w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
              end
            end
        end
    end    
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    currentState        = state;
    jointsSmoothingTime = sm.jointsSmoothingTimes(state);