% computeForwardDynamics
%
% computes the forward dynamics of robot Walkman when the base link is
% fixed, and no other links are in contact with the environment. First, let
% me recall the equations of motion of a multibody system, in contact with
% an environment:
%
%  M*dnu + h = J^t*f + S*tau ----> equations of motion
%  
%  J*dnu + dJ*nu = 0         ----> constraint equations
%
% Let me split the first equation into the floating base equations of
% motion and the joints equations of motion:
%
%  Mb*dnub + Mbj*ddqj + hb = Jb^t*f
%  Mj*ddqj + Mjb*dnub + hj = Jj^t*f + tau
%
% RECALL: dnub = 0 because the base is fixed 
%         f is 0 because the base link is fixed, and no force is
%         transmitted to the rest of the robot
%
% the simplified system is:
%
%  Mbj*ddqj + hb = 0       (1)
%  Mj*ddqj  + hj = tau     (2)
%
% It is then possible to directly calculate tau from (2).
%
% OPTIONAL: in case ddqj measurement is very bad, one can move the joints
% very slowly, and assume that ddqj = 0. This further simplify the
% calculations.
%
function  tau = computeForwardDynamics(M,h,ddqj,USE_h_ONLY)

% compute all required parameters
Mj     = M(7:end,7:end);
hj     = h(7:end);

% special case: the inertia can be neglected
if USE_h_ONLY
    
    ddqj = 0.*ddqj;
end

% compute tau
tau = Mj*ddqj + hj;

end

