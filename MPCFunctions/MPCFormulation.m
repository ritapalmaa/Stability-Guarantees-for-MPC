function [constraints, objective,x,u,G,c,A,b] = MPCFormulation(model,X,U,nx,nu,lqr,N,Xf)
% MPCFormulation - Function that defines the MPC optimization problem 
%
% Syntax:  
%    [constraints, objective,x,u,G,c,A,b] = MPCFormulation(model,X,U,nx,nu,lqr,N,Xf)
%
% Inputs:
%    model - struct with the system model A and B matrices
%    X - Constrained Convex Generator of state constraints
%    U - Constrained Convex Generator of input constraints
%    nx - state dimensions
%    nu - input dimensions
%    lqr - struct that containts the lqr matrices (P,Q,R)
%    N - prediction horizon
%    Xf - the maximum admissible sets to change Xf
% 
% Outputs:
%    constraints - represents a collection of optimization constraints. 
%    objective - objective function that the MPC controller aims to minimize
%    x - predicted state trajectory over N
%    u - predicted control input trajectory over N 
%    G - generator matrix for the variable terminal set (CCG)
%    c - centre for the variable terminal set (CCG)
%    A - linear matrix for the variable terminal set (CCG)
%    b - linear vector for the variable terminal set (CCG)
%
% Other m-files required: compileCCGOptimal.m, compileCCG.m
% Subfunctions: none
% MAT-files required: none 
%
% Notes:
% The u definition does not permit calling u{1}. 
% Therefore, when N = 1 some ajustments are done.

%------------- BEGIN CODE --------------

% Define the optimization variables
u = sdpvar(repmat(nu,1,N),ones(1,N));
x = sdpvar(repmat(nx,1,N+1),ones(1,N+1));

objective = 0; constraints = []; 

for k = 1 : N
    % Compile the sets for the constraints
    if k == N
        [set_X_test, x{N+1}, A,b,G,c] = compileCCGOptimal(Xf);           
    else
        [set_X_test, x{k+1}] = compileCCG(X);
    end

    if N == 1 
        [set_U_test, u] = compileCCG(U);
        % Define the system constraints & cost function 
        objective = objective + x{k}'*lqr.Q*x{k} + u'*lqr.R*u;
        constraints = [constraints, ...
                       x{k+1} == model.A*x{k} + model.B*u, ...
                       set_X_test, set_U_test
                       ]; 
    else
        [set_U_test, u{k}] = compileCCG(U);
        % Define the system constraints & cost function 
        objective = objective + x{k}'*lqr.Q*x{k} + u{k}'*lqr.R*u{k};
        constraints = [constraints, ...
                       x{k+1} == model.A*x{k} + model.B*u{k}, ...
                       set_X_test, set_U_test
                       ]; 
    end

end

% Add the terminal cost
objective = objective + x{N+1}'*lqr.P*x{N+1}; 
end