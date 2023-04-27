%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc] = generate_system_cont(params)
    % YOUR CODE HERE
    w = params.model.GravitationalParameter/(params.model.TargetRadius^3);
    w = sqrt(w);
    m = params.model.Mass;
    syms xddot yddot zddot xdot zdot ydot x y z ux uy uz;
    states = [x, y, z, xdot, ydot, zdot];
    inputs = [ux, uy, uz];
    xddot = 2*w*ydot + 3*(w^2)*x + ux/m;
    yddot = -2*w*xdot + uy/m;
    zddot = -(w^2)*z + uz/m;
    
    sys = [xdot; ydot; zdot; xddot; yddot; zddot];
    Ac = double(subs(jacobian(sys,states),[states inputs],zeros(1,9)));
    Bc = double(subs(jacobian(sys,inputs),[states inputs],zeros(1,9)));
    
end