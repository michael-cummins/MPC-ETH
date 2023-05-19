function lqr_tuning_script()

    params = generate_params();
    x0 = params.model.InitialConditionA;
    
    best_cost = 10;
    while best_cost >= 8
        q = [94; 0.1579; 300; 0.01; 0.1; 0.1];
        noise = [unifrnd(-10,10); unifrnd(-0.1,0.1); unifrnd(-100,100); 
                unifrnd(-0.005, 0.01); unifrnd(-0.09, 1); unifrnd(-0.09, 1)];
        q = q + noise;
        [tuning_struct, iopt] = lqr_tuning(x0, q, params);
        if isnan(iopt)
            continue
        else
            best_cost = tuning_struct(iopt).InputCost;
        end
    end
    
    % Exercise 12 - Simulation
    % Answer = No
    [struct, i] = lqr_tuning(params.model.InitialConditionB, q, params);
    
    save lqr_tuning_script.mat q tuning_struct
end