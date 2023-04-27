%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    H_u = [eye(3);-eye(3)];
    h_u = ones(6,1)*params.constraints.MaxAbsThrust;
    V = params.model.ScalingMatrix;
    Hx = [eye(3), zeros(3);-eye(3), zeros(3)];
    H_x = Hx;
    smax = params.constraints.MaxAbsPositionXZ;
    ymax = params.constraints.MaxAbsPositionY;
    h_x = [smax; ymax; smax; smax; ymax; smax];
end