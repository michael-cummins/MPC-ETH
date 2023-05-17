%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,ctrl_info] = simulate_uncertain(x0, ctrl, Wt, params)
	Nt = params.model.HorizonLength;
    Xt = zeros(params.model.nx, Nt+1);
    Xt(:,1) = x0;
    Ut = zeros(params.model.nu, Nt);
    A = params.model.A;
    B = params.model.B;
    ctrl_info = struct();
    ctrl_info.ctrl_feas = logical([]);

    for i = 1:Nt
        [Ut(:,i), info] = ctrl.eval(Xt(:,i));
        ctrl_info.ctrl_feas(i) = info.ctrl_feas;
        Xt(:,i+1) = A*Xt(:,i) + B*Ut(:,i) + Wt(:,i);
    end
end