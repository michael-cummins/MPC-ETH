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
    system = LTISystem('A', A+B*K);

    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;

    Xp = Polyhedron('A',[H_x; H_u*K], 'b', [h_x; h_u]);
    
    system.x.with('setConstraint');
    system.x.setConstraint = Xp;

    InvSet = system.invariantSet();
    He = InvSet.H;
    H = He(:, 1:params.model.nx);
    h = He(:,params.model.nx+1);
end

