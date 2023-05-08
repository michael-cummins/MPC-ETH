%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, h] = lqr_maxPI(Q,R,params)
	% YOUR CODE HERE
    A = params.model.A;
    B = params.model.B;
    K = -dlqr(A,B,Q,R);
    system = LTIsystem('A', A+B*K);

    xz_max = params.constraints.MaxAbsPositionXZ;
    y_max = params.constraints.MaxAbsPositionY;
    system.x.min = [-xz_max; -y_max; -xz_max];
    system.x.max = [xz_max; y_max; xz_max];
    system.u.min = -params.constraints.MaxAbsThrust;
    system.u.max = params.constraints.MaxAbsThrust;

    InvSet = system.invariantSet();
    H = InvSet.H;
    h = InvSet.h;
end

