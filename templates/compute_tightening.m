%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% YOUR CODE HERE
    epsilon = Polyhedron('A', H_tube, 'b', h_tube);
    X = Polyhedron('A', params.constraints.StateMatrix, 'b', params.constraints.StateRHS);
    U = Polyhedron('A', params.constraints.InputMatrix, 'b', params.constraints.InputRHS);

    X_new = X.minus(epsilon);
    U_new = U.minus(K_tube*epsilon);

    params.constraints.StateMatrix = X_new.A;
    params.constraints.StateRHS = X_new.b;
    params.constraints.InputMatrix = U_new.A;
    params.constraints.InputRHS = U_new.b;
end