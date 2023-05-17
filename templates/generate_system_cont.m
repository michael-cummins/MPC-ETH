%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc] = generate_system_cont(params)
    % YOUR CODE HERE
    w_n = sqrt(params.model.GravitationalParameter/(params.model.TargetRadius^3));
    A_ = zeros(3,6);
    A_(1,1) = 3*w_n*w_n;
    A_(1,5) = 2*w_n;
    A_(2,4) = -2*w_n;
    A_(3,3) = -w_n*w_n;
    Ac = [zeros(3,3), 1*eye(3); A_];
    Bc = [zeros(3,3); (1/params.model.Mass)*eye(3)];
    
end