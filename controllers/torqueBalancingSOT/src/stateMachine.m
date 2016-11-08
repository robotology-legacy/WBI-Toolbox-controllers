function [w_H_b, CoMDes,qDes,constraints,impedances,kpCom,kdCom,currentState,jointsSmoothingTime,...
          desRootRotPosVelAcc,desLFootRotPosVelAcc,desLFootOrigin,desRFootRotPosVelAcc,desRFootOrigin] = ...
     stateMachine(CoM_0, q0, l_sole_CoM,r_sole_CoM,qj, t, ...
                  wrench_rightFoot,wrench_leftFoot,l_sole_H_b, r_sole_H_b, sm,gain)
    %#codegen
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent w_H_r_sole0;
    persistent l_sole_H_b0;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) || isempty(w_H_r_sole0)  || isempty(l_sole_H_b0) 
        state            = sm.stateAt0;
        tSwitch          = 0;
        w_H_fixedLink    = eye(4);
        w_H_r_sole0 = l_sole_H_b/r_sole_H_b;
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
    
    %% TWO FEET BALANCING
    if state == 1 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;

        if t > sm.tBalancing %after tBalancing time start moving weight to the left
           state = 2;
           if sm.demoOnlyRightFoot
                w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
                state = 7;
           end
           
        end
    end

    %% TRANSITION TO THE LEFT FOOT
    if state == 2 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes     = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        impedances = gain.impedances(state,:);
        kpCom      = gain.PCOM(state,:);   
        kdCom      = gain.DCOM(state,:);   

        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        
        CoMError   = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
        
        qDes       = sm.joints.states(state,:)'; % new reference for q
        
        if norm(CoMError(2)) < sm.com.threshold && wrench_rightFoot(3) < sm.wrench.thresholdContactOff
           state = 3; 
           tSwitch = t;
        end
    end

    %% LEFT FOOT BALANCING 
    if state == 3 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes     = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
 
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        desRFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        
        if t > tSwitch + sm.DT 
            state   = 4;
            tSwitch = t;
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 4 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);
        
        qTildeLLeg  = qj(end-11:end-6)-qDes(end-11:end-6);
            
        if norm(qTildeRLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdInContact
            state   = 5;
            tSwitch = t;
        end
    end
    
    %% LOOKING FOR A CONTACT
    if state == 5
        w_H_b       =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if wrench_rightFoot(3) > sm.wrench.thresholdContactOn
            state = 6;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 6 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        
        
        CoMDes      = CoM_0 + sm.com.states(state,:)';         

        
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        if ((norm(l_sole_CoM(1:2)-CoMDes(1:2)) < 10*sm.com.threshold) && sm.yogaAlsoOnRightFoot && (t > tSwitch + sm.tBalancing))
            w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
            state           = 7;
            tSwitch         = t;
        end
    end
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
     
%% TRANSITION TO THE RIGHT FOOT
    if state == 7 
        constraints = [1; 1]; %right foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;


        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
 
        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        
        CoMError    = fixed_link_CoMDes(1:3) - r_sole_CoM(1:3);
        
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        qDes        =  sm.joints.states(state,:)';

        if norm(CoMError(2)) < sm.com.threshold  && wrench_leftFoot(3) < sm.wrench.thresholdContactOff
           state = 8; 
           tSwitch = t;
        end

    end
    
     %% RIGHT FOOT BALANCING 
    if state == 8
        constraints    = [0; 1]; %left foot is no longer a constraints
        w_H_b          =  w_H_fixedLink*r_sole_H_b;

        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
        impedances     = gain.impedances(state,:);
        kpCom          = gain.PCOM(state,:);   
        kdCom          = gain.DCOM(state,:);   
        
        desLFootOrigin =  w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';

        
        if t > tSwitch + sm.DT % yoga
            state   = 9;
            tSwitch = t;
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 9 
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;

        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);
        
        qTildeLLeg  = qj(end-11:end-6)-qDes(end-11:end-6);
            
        if norm(qTildeRLeg)*180/pi < sm.joints.thresholdInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdNotInContact
            state   = 10;
            tSwitch = t;
        end
    end
    
    %% LOOKING FOR A CONTACT
    if state == 10
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;

        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if wrench_leftFoot(3) > sm.wrench.thresholdContactOn 
            state   = 11;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 11
        w_H_b       =  w_H_fixedLink*r_sole_H_b;
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        if t - tSwitch> sm.tBalancing %after tBalancing time start moving weight to the left
           if sm.yogaInLoop
              state = 2; 
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
    
    
    
    
   