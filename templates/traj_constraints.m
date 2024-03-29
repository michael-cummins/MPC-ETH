%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % YOUR CODE HERE
    state_traj = x;
    x = state_traj(1,:);
    z = state_traj(3,:);
    s_max = max(abs(x), abs(z)); % vector 1 x Nt
    s_max = max(max(s_max)); % scalar
    
    y = state_traj(2,:);
    y_max = max(abs(y)); % scalar
    [urow, ucol] = size(u);
    u = reshape(u, [urow*ucol, 1]);
    u_max = max(norm(u, Inf));  %is it a scalar?
    J_u = u'*u; % scalar?

    xdot = state_traj(4,:);
    ydot = state_traj(5,:);
    zdot = state_traj(6,:);
    df_max = sqrt(x(end)^2 + y(end)^2 + z(end)^2); % scalar
    vf_max = sqrt(xdot(end)^2 + ydot(end)^2 + zdot(end)^2); % scalar
    
    umax = params.constraints.MaxAbsThrust;
    smax = params.constraints.MaxAbsPositionXZ;
    ymax = params.constraints.MaxAbsPositionY;
    df = params.constraints.MaxFinalPosDiff;
    vf = params.constraints.MaxFinalVelDiff;

    if s_max <= smax && y_max <= ymax && u_max <= umax && df_max <= df && vf_max <= vf
        traj_feas = true;
    else
        traj_feas = false;
    end
end

