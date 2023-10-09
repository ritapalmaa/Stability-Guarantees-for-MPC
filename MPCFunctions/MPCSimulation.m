function [path,analyse] = MPCSimulation(controller,x0,Nc,model,Xj_list,N,terminal,idx,set_max) 
% MPCSimulation - Function that simulates the operation of a MPC 
% for a given system
%
% Syntax:  
%    [path,analyse] = MPCSimulation(controller,x0,simulation_steps,model,Xj_list,N,terminal,idx,set_max) 
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
%    analyse - records the diagnostics given by optimizer for analysis
%
% Other m-files required: AddZeros.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization
steps = N*Nc-1;
path = x0;  
analyse = []; 
i = 1; % auxiliar variable to know which optimizer run
N_plot = N; 

% Confirmation if the x0 is not already on mRPI
[~, flag] = terminal{x0};
if idx == 1 || flag == 0
    disp('MPC already in the terminal set');
    analyse = [analyse 0];
    return
end

% Simulation for the specified number of steps considering x0 location,
% runs until the specified number of steps are done or MPC reaches mRPI
for kk = length(Xj_list)-idx : steps
    % Case where the number of steps to mRPI < N
    
    if N+kk+1 > length(Xj_list)
        new_Xf = AddZeros(Xj_list{1}, set_max);
%         i = N+kk+1-length(Xj_list)+1;
%             if i > N-1 && idx ~= 2
%                 path_mpc = [path_mpc x(:,N_plot+1)];
%                 return;
%             end
        N_plot = N+1-i;
    % Normal case
    else
        new_Xf = AddZeros(Xj_list{end-N-kk}, set_max);
    end

    % Run the controller with the horizon indentified
    [solutions, diagnostics] = controller{i}{x0,new_Xf.G,new_Xf.c,new_Xf.A,new_Xf.b}; 
    % Solutions
    u = solutions{1}; x = solutions{2};
    % Analyse mpc at each run 
    analyse = [analyse diagnostics];

    if diagnostics == 1
        disp('The problem is infeasible');
        return
    end

    x0 = model.A*x0 + model.B*u(:,1); 

    path = [path x0];  
 
%         x_axis = repmat(kk : kk+N_plot, 1, 1);
%         subplot(1,3,1); plot3(x_axis,x(1,:),x(2,:),'-x','LineWidth',2,'HandleVisibility','Off');
%         subplot(1,3,2); plot3(x_axis,x(3,:),x(4,:),'-x','LineWidth',2,'HandleVisibility','Off');
%         subplot(1,3,3); plot3(x_axis,x(5,:),x(6,:),'-x','LineWidth',2,'HandleVisibility','Off');

%     subplot(1,3,1); plot(x(1,:),x(2,:),'-x','LineWidth',2,'HandleVisibility','Off');
%     subplot(1,3,2); plot(x(3,:),x(4,:),'-x','LineWidth',2,'HandleVisibility','Off');
%     subplot(1,3,3); plot(x(5,:),x(6,:),'-x','LineWidth',2,'HandleVisibility','Off');

    % Confirmation if the state already reached the mRPI in each run
    [~, flag] = terminal{x0};
    if flag == 0
        disp('MPC already in the terminal set');
        return
    end
end  
end