function [path_mpc,analyse_mpc_inside] = MPCSimulation(controller,x0,simulation_steps,model,Xj_list,N,terminal,idx,set_max) 
% MPCSimulation function runs the mpc trajectory
    path_mpc = x0;  
    analyse_mpc_inside = []; 
    
    N_plot = N; i = 1;
   
    [~, flag] = terminal{x0};
    if idx == 1 && flag == 0
        disp('MPC already in the terminal set');
        analyse_mpc_inside = [analyse_mpc_inside 0];
        return
    end
    
    for kk = length(Xj_list)-idx : simulation_steps
        if N+kk+1 > length(Xj_list)
            new_Xf = AddZeros(Xj_list{1}, set_max);
            i = N+kk+1-length(Xj_list)+1;
            if i > N-1 && idx ~= 2
                path_mpc = [path_mpc x(:,N_plot+1)];
                return;
            end
            N_plot = N+1-i;
        else
            new_Xf = AddZeros(Xj_list{end-N-kk}, set_max);
        end
        
        % Run the controller with the maximum horizon expected
        [solutions, diagnostics] = controller{i}{x0,new_Xf.G,new_Xf.c,new_Xf.A,new_Xf.b}; 
        % Solutions
        u = solutions{1}; x = solutions{2};
        % Analyse mpc at each instant 
        analyse_mpc_inside = [analyse_mpc_inside diagnostics];
        
        if diagnostics == 1
            disp('The problem is infeasible');
            return
        end

        x0 = model.A*x0 + model.B*u(:,1); 
        
        path_mpc = [path_mpc x0];  
        
%         x_axis = repmat(kk : kk+N_plot, 1, 1);
%         subplot(1,3,1); plot3(x_axis,x(1,:),x(2,:),'-x','LineWidth',2,'HandleVisibility','Off');
%         subplot(1,3,2); plot3(x_axis,x(3,:),x(4,:),'-x','LineWidth',2,'HandleVisibility','Off');
%         subplot(1,3,3); plot3(x_axis,x(5,:),x(6,:),'-x','LineWidth',2,'HandleVisibility','Off');
        
        [~, flag] = terminal{x0};
        if flag == 0
            disp('MPC already in the terminal set');
            return
        end
    end  
end