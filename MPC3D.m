% CODE REVIEWED ON 03/01
close all;
%%
yalmip('clear')
clc, clear;

addpath('CCGFunctions');
addpath('MPCFunctions');
addpath('Functions');

% Code's parameters
obst_introd = 0; % 0-off or 1-on
idx_obs = 4; % index where obstacle is introduced
reduction = 1; % 0-off or 1-on
%% MPC3D - Drone
% Initialization
nx = 9; nu = 3; N = 4;
convergence_steps = 3; 
simulation_steps = N*convergence_steps-1;
Ts = 0.1; %[s]
[model,X,U,D,RPI,lqr,Acl] = InitDrone(nx,nu,Ts,simulation_steps);
%%
if obst_introd == 1
    Obs.G = 1*diag([1 1 1 1.5 1.5 .5 2 2 2]); Obs.c = zeros(nx,1); 
    Obs.A = zeros(0,nx); Obs.b = zeros(0,1); 
    Obs.type = Inf; Obs.idx = nx;
    [set_Obs,a_Obs] = compileCCG(Obs); 
end

% Compute Region of Attraction
Xj_list = BackwardReachabilitySets(RPI,X,U,model,nx,convergence_steps,simulation_steps,N);

Xj_pre = Xj_list{end}; % set used for plots
disp('Region of attraction computed.');

% Region of Attraction reduced
if reduction == 1
    maxError= 1e-6;
    Xj_list{end} = SetReduction(maxError,Xj_list{end},nx,nu);
    disp('Region of attraction reduced.');
end

% Compile Sets
[set_Xj,a_Xj] = compileCCG(Xj_list{end}); 
[set_RPI,a_RPI] = compileCCG(RPI); 

% Plot Set Reverse
PlotGeral(Xj_pre,Xj_list,nx,nu,N);

set_max = Xj_list{end};
% Introduction of obstacles & computation of the new sequence of sets
if obst_introd == 1    
    Xj_list_obs = ForwardReachabilitySets(idx_obs,model,X,U,Xj_list,Obs);
    PlotSetObstacles3D(Xj_list_obs,nx,nu,N);
    legend('X_{j-h}, h = 1, ...,N_c','X_{i_{j-h}}, i = 1, ..., N','O_{j-h}, h = 1, ...,N_c','O_{i_{j-h}}, i = 1, ..., N');
    Xj_list = Xj_list_obs;
    set_max = Xj_list_obs{1};
    disp('Obstacles introduced');
end

ops = sdpsettings('solver','MOSEK','verbose',1); ops2 = sdpsettings('solver','GUROBI');
% Optimizer to search initial conditions 
vector = sdpvar(nx,1); 
in = {vector}; out = {a_Xj};
boundary = optimizer(set_Xj,vector'*a_Xj,ops2,in,out);
confirmation = optimizer([set_Xj, a_Xj == vector],0,ops2,in,out);

% Optimizer MPC controllers
controller = cell(1,N);
for i = 0 : N-1
    horizon = N-i;
    [constraints,objective,x,u,G_in,c_in,A_in,b_in] = MPCFormulation(model,X,U,nx,nu,lqr,horizon,set_max);
    parameters_in = {x{1},G_in,c_in,A_in,b_in}; 
    if horizon == 1
        solutions_out = {u, [x{:}]};
    else 
        solutions_out = {[u{:}], [x{:}]};
    end
    mpc = optimizer(constraints,objective,ops,parameters_in,solutions_out);
    controller{i+1} = mpc;
end

% Optimizer to confirm location in RPI
out = {a_RPI};
terminal = optimizer([set_RPI, a_RPI == vector],0,ops2,in,out);

disp('Optimizers computed.');
%% Run MPC for different vertices in the Region of Atraction
n_sim = 1; % number of Monte Carlo runs
j = 0;
min = 0.07; max = 0.14; % limits to look for a initial condition
traj_symb = {'-o','-s'};

% Analysis variables
init = [];
analyse_mpc = cell(1,n_sim);
n_times_incorrect = 0;

while size(init,2) < n_sim
    dif_zero = 0; % count number of diagnostics diff 0
    j = j+1;
    
    vector = -min + max*rand(nx,1);
    % Find vertex with mode 1. or 2.
    % 1. Boundary of RA
    vector = vector./norm(vector); [vertex, ~] = boundary{vector};
    % 2. Inside RA
%     [vertex, ~] = confirmation{vector};
    
    % the optimal value to minimise the cost function is the origin, 
    % the if is added to consider multiple vertices
    if vertex ~= zeros(nx,1) 
        % Define the initial condition
        x0 = vertex; %+ 0.05*vector; %can add a perturbance or not
        init = [init x0]; 
      
        % Analyse of the initial condition
        [~, flag] = confirmation{x0};
        if flag ~= 0 
             disp('Not a vertex.')
        end
        
        % Find the larger set in which x0 belongs
        idx = FindSet(Xj_list,x0);
        
        % Run the MPC with the initial condition x0
        [path_mpc,analyse_mpc_inside] = MPCSimulation(controller,x0,simulation_steps,model,Xj_list,N,terminal,idx,set_max);
        
        % Analyse of diagnostics diff 0 in analyse_mpc_inside
        analyse_mpc{j} = analyse_mpc_inside(end);
        for i = 1 : length(analyse_mpc_inside)
            if analyse_mpc_inside(i) ~= 0
                dif_zero = dif_zero+1;
            end
        end
        
        t = 1; % index for trajectory symbol
        if dif_zero ~=0
            n_times_incorrect = n_times_incorrect +1;
            t = 2; % index for trajectory symbol
        end
        
        subplot(1,3,1);
        plot3(path_mpc(1,:),path_mpc(2,:),path_mpc(3,:),traj_symb{t},'LineWidth',2,'HandleVisibility','Off');
        subplot(1,3,2);
        plot3(path_mpc(4,:),path_mpc(5,:),path_mpc(6,:),traj_symb{t},'LineWidth',2,'HandleVisibility','Off');
        subplot(1,3,3);
        plot3(path_mpc(7,:),path_mpc(8,:),path_mpc(9,:),traj_symb{t},'LineWidth',2,'HandleVisibility','Off');
        
    end
end
fprintf('Number of incorrect simulations %i \n',n_times_incorrect);