function ddx_CoM_bounds = CoMaccelerationBounds( w_H_CoM, CoMVelocity, ...
                           w_H_l_sole, w_H_r_sole, ...
                           gain )

%% NOTE: definition of position bounds shall be redefined for walking purposes
%e.g. can add feet velocity in the definition of the acceleration bounds
%e.g. define stable position as the predicted support polygon

%CoM current position and velocity, in x- and y-axis
x_CoM    = w_H_CoM(1:2,4);
dx_CoM   = CoMVelocity(1:2);                       

%Feet current position, in x- and y-axis
x_l_sole = w_H_l_sole(1:2,4);
x_r_sole = w_H_r_sole(1:2,4);

%% Bounds on CoM position, for 2 feet balancing
x_CoM_lowerBound = min(x_r_sole, x_l_sole); % + [gain.footSize(2,1); gain.footSize(1,1)]*0;
x_CoM_upperBound = max(x_r_sole, x_l_sole); % + [gain.footSize(2,2); gain.footSize(1,2)]*0;

ddx_CoM_lowerBound = min(linearPID(x_CoM, dx_CoM, [x_CoM_lowerBound zeros(2,2)], gain.CoMboundFactor * [gain.x_CoM.p, gain.x_CoM.d]), ...
                         linearPID(x_CoM, dx_CoM, [x_CoM_upperBound zeros(2,2)], gain.CoMboundFactor * [gain.x_CoM.p, gain.x_CoM.d]));
ddx_CoM_upperBound = max(linearPID(x_CoM, dx_CoM, [x_CoM_lowerBound zeros(2,2)], gain.CoMboundFactor * [gain.x_CoM.p, gain.x_CoM.d]), ...
                         linearPID(x_CoM, dx_CoM, [x_CoM_upperBound zeros(2,2)], gain.CoMboundFactor * [gain.x_CoM.p, gain.x_CoM.d]));

ddx_CoM_bounds = [ddx_CoM_lowerBound; ddx_CoM_upperBound];

end

