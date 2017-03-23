function [w_H_b,constraints,impedances,kpCom,kdCom,currentState,jointsAndCoMSmoothingTime,qjDes,CoM_Des] = ...
          stateMachineStandUp(qjRef,CoM,CoM_0,l_sole_H_b,l_upper_leg_contact_H_b,t,gain,sm,Lwrench,Rwrench)
      
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent CoMprevious;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) || isempty(CoMprevious)
        state         = sm.stateAt0;
        tSwitch       = 0;
        w_H_fixedLink = eye(4);
        CoMprevious   = CoM_0;
    end

    constraints               = [1;1];
    w_H_b                     = eye(4);
    impedances                = gain.impedances(1,:);
    kpCom                     = gain.PCOM(1,:);   
    kdCom                     = gain.DCOM(1,:);
    qjDes                     = qjRef;
    CoM_Des                   = CoM_0;
    jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
    
    %% BALANCING ON THE LEGS
    if state == 1 
        
        w_H_b                     =  w_H_fixedLink * l_upper_leg_contact_H_b;
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        
        % after tBalancing time, start moving CoM forward
        if t > sm.tBalancing 
            state = 2;           
        end
    end

    %% MOVE COM FORWARD
    if state == 2 
        
        w_H_b                =  w_H_fixedLink * l_upper_leg_contact_H_b;
        
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
        
        CoM_Des                   = CoM_0 + transpose(sm.CoM.standUpDeltaCoM(state,:));
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        
        if (Lwrench(3)+Rwrench(3)) > (sm.LwrenchTreshold(state) + sm.RwrenchTreshold(state))
            state           = 3;
            w_H_fixedLink   = w_H_fixedLink * l_upper_leg_contact_H_b/l_sole_H_b;
            tSwitch         = t;
            CoMprevious     = CoM;
        end
        
    end

    %% TWO FEET BALANCING
    if state == 3 
        
        w_H_b                =  w_H_fixedLink * l_sole_H_b;

        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
        qjDes(16)            = sm.joints.leftAnkleCorrection;
        
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));
        tDelta                    = t-tSwitch;
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        
        if Lwrench(3) > sm.LwrenchTreshold(state) &&  Rwrench(3) > sm.RwrenchTreshold(state) && tDelta > 1
            state       = 4;
            CoMprevious = CoM;
        end
    end
    
    %% LIFTING UP
    if state == 4 
        
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
          
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
        
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));    
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
    end

    currentState = state;
    
end
 
   