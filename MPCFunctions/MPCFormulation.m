function [constraints, objective,x,u,G,c,A,b] = MPCFormulation(model,X,U,nx,nu,lqr,N,Xf)
% mpc_formulation function defines the optimizer parameters for 
% the mpc formulation

    % Define the optimization variables
    u = sdpvar(repmat(nu,1,N),ones(1,N));
    x = sdpvar(repmat(nx,1,N+1),ones(1,N+1));

    objective = 0; constraints = []; 
    for k = 1 : N 
        % Compile the sets for the constraints
        if k == N
            [set_X_test, x{N+1}, A,b,G,c] = compileCCG_opt(Xf);           
        else
            [set_X_test, x{k+1}] = compileCCG(X);
        end
                
        [set_U_test, u{k}] = compileCCG(U);
                
        % Define the system constraints & cost function 
        objective = objective + x{k}'*lqr.Q*x{k} + u{k}'*lqr.R*u{k};
        constraints = [constraints, ...
                       x{k+1} == model.A*x{k} + model.B*u{k}, ...
                       set_X_test, set_U_test
                       ];       
    end
    
    % Define the terminal cost and constraint
    objective = objective + x{N+1}'*lqr.P*x{N+1}; 
end