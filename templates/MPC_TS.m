%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS(Q,R,N,H,h,params)
            nu = params.model.nu;
            nx = params.model.nx;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');

            % YOUR CODE HERE
            A = params.model.A;
            B = params.model.B;
            [P,~,~] = idare(A,B,Q,R,[],[]);
            
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            

            constraints = [];
            objective = 0;
            x = X0;
            for k = 1:N
                 objective = objective + x'*Q*x + U{k}'*R*U{k};
                 constraints = [constraints, Hu*U{k} <= hu, Hx*x <= hx];
                 x = A*x + B*U{k};
            end
            objective =  objective + x'*P*x;
            constraints = [constraints, H*x<=h];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end