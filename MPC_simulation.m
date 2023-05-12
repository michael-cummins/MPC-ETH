close all;
clear;
clc;

params = generate_params();
Q = diag([94.0; .1579; 300.0; 0.01; 0.1; 0.1]);
R = eye(params.model.nu);
N = 30;

x0A = params.model.InitialConditionA;
ctrl_MPC = MPC(Q, R, N, params);
ctrl_MPC_TE = MPC_TE(Q, R, N, params);

[H, h] = lqr_maxPI(Q,R,params);
ctrl_MPC_TS = MPC_TS(Q, R, N, H, h, params);

[x, u, ctrl_info] = simulate(x0A, ctrl_MPC, params);
[fig_time,axes_time,fig_pos,axes_pos] = plot_trajectory(x,u,ctrl_info,params)

[x, u, ctrl_info] = simulate(x0A, ctrl_MPC_TE, params);
[fig_time,axes_time,fig_pos,axes_pos] = plot_trajectory(x,u,ctrl_info,params)

[x, u, ctrl_info] = simulate(x0A, ctrl_MPC_TS, params);
[fig_time,axes_time,fig_pos,axes_pos] = plot_trajectory(x,u,ctrl_info,params)