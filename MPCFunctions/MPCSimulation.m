function path = MPCSimulation(controller,x0,Nc,model,Xj_list,N,terminal,idx,set_max) 
% MPCSimulation - Function that simulates the operation of a MPC 
% for a given system
%
% Syntax:  
%    path = MPCSimulation(controller,x0,simulation_steps,model,Xj_list,N,terminal,idx,set_max) 
%
% Inputs:
%    controller - cell with YALMIP optimizers with different horizons
%    x0 - initial state
%    Nc - number of convergence steps
%    model - struct with the system model A and B matrices
%    Xj_list - sequence of reachable sets 
%    N - prediction horizon
%    idx - index in the sequence where x0 belongs
%    set_max - the maximum admissible sets to change Xf
%
% Outputs:
%    path - state trajectory
%
% Other m-files required: AddZeros.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization
path = x0;  
i = 1; % auxiliar variable to know which optimizer run

% Confirmation if the x0 is not already on mRPI
[~, flag] = terminal{x0};
if flag == 0
    disp('MPC already in the terminal set');
    return
end

% Simulation for the specified number of steps considering x0 location
for kk = 1  : N*Nc+1 %kk = 1 : N*Nc %kk = length(Xj_list)-idx+1 
    % Define the terminal set
    j = max(length(Xj_list)-kk-N+1,1); 
    new_Xf = AddZeros(Xj_list{j}, set_max); 
    
    % Define the controller
%     if length(Xj_list)-kk-N+1 < 1
%         i = min(i+1,N);
%     end
%     
%     fprintf('kk=%i ',kk); fprintf('j=%i ',j); fprintf('i=%i\n ',i);
    
    % Run the controller with the horizon indentified
    [solutions, diagnostics] = controller{i}{x0,new_Xf.G,new_Xf.c,new_Xf.A,new_Xf.b} 
    
    % Solutions
    u = solutions{1}; x = solutions{2};
    
    % Analyse mpc at each step 
    if diagnostics == 1
        disp('The problem is infeasible');
        return
    end
        
    % Compute next state and add to the trajectory
    x0 = model.A*x0 + model.B*u(:,1); 
    path = [path x0];  
        
    % Confirmation if the state already reached the mRPI in each run
    [~, flag] = terminal{x0};
    if flag == 0
        disp('MPC already in the terminal set');
        return
    end
end  
end