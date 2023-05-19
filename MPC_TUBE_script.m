function MPC_TUBE_script()

    params = generate_params_z();
    x0 = params.model.InitialConditionC;
    
    Wt = generate_disturbances(params);

    U_SC = zeros(params.model.nu, params.model.HorizonLength);
    U = ones(params.model.nu, params.model.HorizonLength);
    
    A = params.model.A;
    B = params.model.B; 
    Q = diag([300, 0.1]);
    R = 1;
    N = 50;
    p = [0.05, 0.1];

    K_tube = compute_tube_controller(p, params);

    [H, h] = lqr_maxPI(Q,R, params);
    
    mpc_tube = MPC_TUBE(Q,R,N,H,h,params);
    [ne,~] = size(params.constraints.StateRHS);
    
    x = x0;
    S =10 *eye(ne);
    v =100;
    mpc_TS_SC = MPC_TS_SC(Q,R,N,H,h,S,v,params);
    
   
    for i = 1:params.model.HorizonLength
        i
        [U(:,i), ctrl_info] = mpc_TS.eval(x);
        [U_SC(:,i), ~] = mpc_TS_SC.eval(x);
        if ctrl_info.ctrl_feas
            
            if U(:,i) - U_SC(:,i) > 1.0e-07
                r = 'cest pas les memes'
                break
            end
        
        else 
            r = 'unfeasible'
            break
        end
        x = A*x + B*U(:,i);
    end
    clear
    save MPC_TUBE_params.mat S v 
end
