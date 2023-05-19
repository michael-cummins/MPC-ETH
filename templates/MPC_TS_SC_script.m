function MPC_TS_SC_script()

    params = generate_params();
    x0 = params.model.InitialConditionC;
    
    U_SC = zeros(params.model.nu, params.model.HorizonLength);
    U = ones(params.model.nu, params.model.HorizonLength);
    
    A = params.model.A;
    B = params.model.B; 
    Q = diag([94, 0.1579, 300, 0.01, 0.1, 0.1]);
    R = eye(params.model.nu);
    N = 30;
    [H, h] = lqr_maxPI(Q,R, params);
    mpc_TS = MPC_TS(Q,R,N,H,h,params);
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
    save MPC_TS_SC_params.mat S v 
end
