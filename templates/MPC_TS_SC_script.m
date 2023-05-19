close all;
clear;
clc;

params = generate_params();
x0 = params.model.InitialConditionA;

U_SC = zeros(params.model.nu, params.model.HorizonLength);
U = ones(params.model.nu, params.model.HorizonLength);

A = params.model.A;
B = params.model.B; 
Q = diag([94; 0.1579; 300; 0.01; 0.1; 0.1]);
R = eye(params.model.nu);
N = params.model.HorizonLength;
[H, h] = lqr_maxPI(Q,R, params);
mpc_TS = MPC_TS(Q,R,N,H,h,params);
[ne,~] = size(params.model.StateRHS);

x = x0;
S = diag(1000*ones(ne));
v = 1000;
mpc_TS_SC = MPC_TS_SC(Q,R,N,H,h,S,v,params);

same = true;
for i = 1:params.model.HorizonLength
    [U(:,i), ctrl_info] = mpc_TS.eval(x);
    [U_SC(:,i), ctrl_SC_info] = mpc_TS_SC.eval(x);
    if ctrl_info.feasible
        if U(:,i) ~= U_SC(:,i)
            same = false;
            break
        end
    end
    if ctrl_info.feasible == false
        same = false;
        break
    end
    x = A*x + B*U(:,i);
end

same