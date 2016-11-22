function [w_H_b,constraints,impedances,kpCom,kdCom,currentState,jointsSmoothingTime,qjDes,CoM_Des] = ...
          stateMachineChair(qjRef,CoM_0,l_sole_H_b,l_leg_H_b,t,gain,sm,Lwrench,Rwrench)
      
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent tInitState3;

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
    qjDes       = qjRef;
    CoM_Des     = CoM_0;
    tInitState3 = 0;

    %% BALANCING ON THE LEGS
    if state == 1 
        w_H_b      =  w_H_fixedLink * l_leg_H_b;

        if t > sm.tBalancing % after tBalancing time start moving CoM forward
           
            state = 2;           
        end
    end

    %% COM TRANSITION
    if state == 2 
        
        w_H_b      =  w_H_fixedLink * l_leg_H_b;
                               
        qjDes([18 19 21 22]) = sm.joints.statesChair(state-1,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.statesChair(state-1,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.statesChair(state-1,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.statesChair(state-1,[5 6 7 8]);
        qjDes(1)             = sm.joints.statesChair(state-1,9);
        CoM_Des              = sm.CoM.statesChair(state-1,:)';
        
        if Lwrench(3) > sm.LwrenchTreshold(state-1) &&  Rwrench(3) > sm.RwrenchTreshold(state-1)
           
            state           = 3;
            w_H_fixedLink   = w_H_fixedLink*l_leg_H_b/l_sole_H_b;
            tInitState3     = t;
        end
        
    end

    %% LOOKING FOR CONTACT
    if state == 3 
        
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
         
        qjDes([18 19 21 22]) = sm.joints.statesChair(state-1,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.statesChair(state-1,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.statesChair(state-1,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.statesChair(state-1,[5 6 7 8]);
        qjDes(1)             = sm.joints.statesChair(state-1,9);
        CoM_Des              = sm.CoM.statesChair(state-1,:)';
        tDelta               = t-tInitState3;
        
        if Lwrench(3) > sm.LwrenchTreshold(state-1) &&  Rwrench(3) > sm.RwrenchTreshold(state-1) && tDelta > 1
            
            state = 4;
        end
    end
    
    %% TWO FEET BALANCING
    if state == 4 
        
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
                               
        qjDes([18 19 21 22]) = sm.joints.statesChair(state-1,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.statesChair(state-1,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.statesChair(state-1,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.statesChair(state-1,[5 6 7 8]);
        qjDes(1)             = sm.joints.statesChair(state-1,9);
        CoM_Des              = sm.CoM.statesChair(state-1,:)';    
    end

currentState        = state;
jointsSmoothingTime = sm.jointsSmoothingTimes(state);
    
end
 
   