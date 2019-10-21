function tau = completeForceControlWithSubTasks(M,h,star,JDotqDot,J)
% This function implements the complete force controle presented in Del
% Prete PhD thesis, pg. 86, equation (6.27). Given the floating base system
% in the form
%       
%       M(q) dot(V) + h(q,v) - Jc^\top f = S tau
%
% We want to find tau such that f = f^*, and with the remaining "degrees of
% actuation" we want to achieve n tasks characterized by the associated
% Jacobians.

    n         = size(M,1) - 6;
    N         = size(J,1);        %number of tasks
    Mb        = M(1:6,1:6);
    Mbj       = M(1:6,7:end);
    Mj        = M(7:end,7:end);
    Jc        = J{end};
    qDDStar_o = star{1};
    fStar     = star{end};
    Jcb       = Jc(:,1:6);
    
    Nj        = inv(Mj - Mbj'/Mb*Mbj);
    Sbar      = [ -Mb\Mbj
                eye(n) ]; 
            
    U         = [eye(6) zeros(6,n)];
           
    JcSbar    = Jc*Sbar;    
    
    qDDStar   = zeros(n,1);
    hb        = h(1:6);
    
    for i = N:-1:2
        if i == N               % I write this initialization in this way just to follow exactly pg. 86, equation (6.27)
            Npi      = eye(n);  % Otherwise, it suffices to put the folling line under qDDStar, but i+1 -> i. Then, this "if" can be taken off  
            xDDStari = 0; 
        else
            Npi      = Npi - pinv(J{i+1}*Sbar*Npi)*J{i+1}*Sbar*Npi;
            xDDStari = star{i};
        end
        
        qDDStar = qDDStar + pinv(J{i}*Sbar*Npi)*(xDDStari - JDotqDot{i} + J{i}*(U'/Mb*(hb - Jcb'*fStar) - Sbar*qDDStar));
        
    end
    
    Npo      = Npi - pinv(J{2}*Sbar*Npi)*J{2}*Sbar*Npi;
    qDDStar  = qDDStar + Npo*qDDStar_o; 
    
    tau      = -(JcSbar)'*fStar + Nj\qDDStar + Sbar'*h;

end

