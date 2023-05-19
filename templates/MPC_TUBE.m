%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params)
            obj.K_tube = K_tube;

            nu = params.model.nu;
            nx = params.model.nx;

            % define optimization variables
            V = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            Z = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            
            X0 = sdpvar(nx,1,'full');
            % YOUR CODE HERE
            A = params.model.A;
            B = params.model.B;
            [P,~,~] = idare(A,B,Q,R,[],[]);
            
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            

            constraints = [H_tube*(X0-Z{1}) <= h_tube];
            objective = 0;
                       
            for k = 1:N
                 objective = objective + Z{k}'*Q*Z{k} + V{k}'*R*V{k};
                 constraints = [constraints, ...
                                Hu*V{k} <= hu, ...
                                Hx*Z{k} <= hx, ...
                                Z{k+1} == A*Z{k} + B*V{k}];
            end
  
            objective =  objective + Z{N+1}'*P*Z{N+1};
            constraints = [constraints, H_N*Z{N+1} <= h_N];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{V{1} Z{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            % YOUR CODE HERE
            [v,z,objective] = optimizer_out{:};
            u = v + obj.K_tube*(x-z);
            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end