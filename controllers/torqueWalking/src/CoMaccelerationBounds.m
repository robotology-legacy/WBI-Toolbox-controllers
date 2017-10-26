function ddx_root_bounds = CoMaccelerationBounds( w_H_root, rootVelocity, ...
                           w_H_l_sole, w_H_r_sole, feetActivation, ...
                           gain )

%% NOTE: definition of position bounds shall be redefined for walking purposes
%e.g. can add feet velocity in the definition of the acceleration bounds
%e.g. define stable position as the predicted support polygon

%root current position and velocity, in x- and y-axis
x_root   = w_H_root(1:2,4);
dx_root  = rootVelocity(1:2);                       

%Feet current position, in x- and y-axis
x_l_sole = w_H_l_sole(1:2,4);
x_r_sole = w_H_r_sole(1:2,4);

%% Bounds on root position, for 2 feet balancing
if feetActivation(1) > 0.9 && feetActivation(2) > 0.9
    x_root_lowerBound   = min(x_r_sole, x_l_sole) + [gain.footSize(1,1); gain.footSize(2,1)];
    x_root_upperBound   = max(x_r_sole, x_l_sole) + [gain.footSize(1,2); gain.footSize(2,2)];
    
elseif feetActivation(2) < 0.9 %only left foot on ground
    x_root_lowerBound   = x_l_sole + [gain.footSize(1,1); gain.footSize(2,1)];
    x_root_upperBound   = x_l_sole + [gain.footSize(1,2); gain.footSize(2,2)];
    
else %only right foot on ground
    x_root_lowerBound   = x_r_sole + [gain.footSize(1,1); gain.footSize(2,1)];
    x_root_upperBound   = x_r_sole + [gain.footSize(1,2); gain.footSize(2,2)];
end

ddx_root_lowerBound = gain.x_maxAcceleration * tanh( min( ...
                      linearPID(x_root, dx_root, [x_root_lowerBound zeros(2,2)], [gain.x_rootbound.p, gain.x_rootbound.d]), ...
                      linearPID(x_root, dx_root, [x_root_upperBound zeros(2,2)], [gain.x_rootbound.p, gain.x_rootbound.d])) );
ddx_root_upperBound = gain.x_maxAcceleration * tanh( max( ...
                      linearPID(x_root, dx_root, [x_root_lowerBound zeros(2,2)], [gain.x_rootbound.p, gain.x_rootbound.d]), ...
                      linearPID(x_root, dx_root, [x_root_upperBound zeros(2,2)], [gain.x_rootbound.p, gain.x_rootbound.d])) );


ddx_root_bounds     = [ddx_root_lowerBound; ddx_root_upperBound];

end

