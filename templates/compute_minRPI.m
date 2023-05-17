%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    A = params.model.A;
    B = params.model.B;

    nx = params.model.nx;

    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;

    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;

    F = [H_x; H_u*K_tube];
    f = [h_x; h_u];

    X = Polyhedron(F, f);
    W = Polyhedron(H_w, h_w);
   
    
    
    omega_new = X.copy();
    
    n_iter = 0;
    
    while false
        n_iter = n_iter + 1;
        omega_old = omega_new.minus(W);
        
        preW = Polyhedron(omega_old.A*(A-K_tube*B),omega_old.b);

        omega_new = omega_old.intersect(preW);
        omega_new = omega_new.minHRep();

        if omega_new == omega_old
            break
        end
    end
   
    H_tube = omega_new.A;
    h_tube = omega_new.b;
end