function MPC_TUBE_script()
    params = generate_params();
    params_z = generate_params_z(params);
    x0 = params_z.model.InitialConditionA_z;
    
    Wt = generate_disturbances(params_z);
    U = ones(params_z.model.nu, params_z.model.HorizonLength);
    
    A = params_z.model.A;
    B = params_z.model.B; 
    Q = diag([300, 0.1]);
    R = 1;
    N = 50;
    p = [0.05, 0.1];

    K_tube = compute_tube_controller(p, params_z);
    
    [H_tube, h_tube] = compute_minRPI(K_tube, params_z);

    params_z_tube = compute_tightening(K_tube, H_tube, h_tube, params_z);

    [H_N, h_N] = lqr_maxPI(Q,R, params_z_tube);
    mpc_tube = MPC_TUBE(Q,R,N,H_N,h_N,H_tube, h_tube, K_tube, params_z_tube);
   
    x = x0;
    for i = 1:params.model.HorizonLength
        [U(:,i), ctrl_info] = mpc_tube.eval(x);
        if ctrl_info.ctrl_feas == false
            break
        end
        x = A*x + B*U(:,i) + Wt(:,i);
    end
    
    save MPC_TUBE_params.mat p K_tube H_tube h_tube H_N h_N params_z_tube

end
