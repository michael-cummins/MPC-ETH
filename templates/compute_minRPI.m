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
    Ak = A+B*K_tube;

    nx = params.model.nx;

    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;

    W = Polyhedron('A',H_w,'b',h_w);

    omega_new = Polyhedron.emptySet(nx);
   
    n_iter = 0;
    
    while true
        omega_old = omega_new.copy();
        omega_old = omega_old.minHRep();
    
        omega_new = omega_old + mtimes(Ak^n_iter,W);
        
        omega_new = omega_new.minHRep();
        
        n_iter = n_iter + 1;
        if eq(omega_new, omega_old)
            break
        end
    end
    
    H_tube = omega_new.A;
    h_tube = omega_new.b;
end