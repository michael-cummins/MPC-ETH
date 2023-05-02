close all;
clear;
clc;

params = generate_params();
Q = diag([94.0; .1579; 300.0; 0.01; 0.1; 0.1]);
R = eye(params.model.nu);
ctrl = LQR(Q, R, params);
x0A = params.model.InitialConditionA;
x0B = params.model.InitialConditionB;
x0C = params.model.InitialConditionC;
[x, u, ctrl_info] = simulate(x0C, ctrl, params);
[fig_time,axes_time,fig_pos,axes_pos] = plot_trajectory(x,u,ctrl_info,params)