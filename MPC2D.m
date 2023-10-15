% CODE REVIEWED ON 03/01
close all;
%%
yalmip('clear')
clc, clear;

addpath('CCGFunctions');
addpath('MPCFunctions');
addpath('Functions');
%%
% Code's parameters
obst_flag = 0; % 0-off or 1-on
idx_obs = 11; % index where obstacle is introduced
reduction = 1; % 0-off or 1-on
n_dimensions = 2; % 2-dimensions or 3-dimensions plot
%% MPC2D
% Initialization
nx = 6; nu = 2; 
N = 4; Nc = 3; 
steps = N*Nc-1;
Ts = 0.1; %[s]
[model,X,U,D,RPI,lqr,Acl] = Init2D(nx,nu,Ts,steps);

if obst_flag == 1
    Obs.G = 1*diag([0.45 0.45 200 200 200 200]); Obs.c = zeros(nx,1); 
    Obs.A = zeros(0,nx); Obs.b = zeros(0,1); 
    Obs.type = Inf; Obs.idx = nx;
    [set_Obs,a_Obs] = compileCCG(Obs); 
end 

% Compute Region of Attraction
Xj_list = BackwardReachabilitySets(RPI,X,U,model,nx,Nc,N);

Xj_pre = Xj_list{end}; % set used for plots
disp('Region of attraction computed.');

% Region of Attraction reduced
if reduction == 1
    maxError = 1e-6;
    Xj_list{end} = SetReduction(maxError,Xj_list{end},nx,nu);
    disp('Region of attraction reduced.');
end

% Compile Sets
[set_Xj,a_Xj] = compileCCG(Xj_list{end}); 
[set_RPI,a_RPI] = compileCCG(RPI); 

% Plot Set Reverse
if obst_flag == 0 && n_dimensions == 2
    PlotGeral(Xj_pre,Xj_list,nx,nu,N);
elseif obst_flag == 0 && n_dimensions == 3
    Plot2DIn3D(Xj_pre,Xj_list,nx,nu,N);
end

set_max = Xj_list{end};
% Introduction of obstacles & computation of the new sequence of sets
if obst_flag == 1  
    Xj_list_obs = ForwardReachabilitySets(idx_obs,model,U,Xj_list,Obs);
    PlotSetObstacles(Xj_list, Xj_list_obs,N);
    Xj_list = Xj_list_obs;
    set_max = Xj_list_obs{1};
    disp('Obstacles introduced');
end

ops = sdpsettings('solver','mosek'); ops2 = sdpsettings('solver','gurobi');%,'gurobi.ObjScale',-0.5);
% Optimizer to search initial conditions 
vector = sdpvar(nx,1); 
in = {vector}; out = {a_Xj};
boundary = optimizer(set_Xj,vector'*a_Xj,ops2,in,out);
confirmation = optimizer([set_Xj, a_Xj == vector],0,ops2,in,out);

% Optimizer to confirm location in RPI
out = {a_RPI};
terminal = optimizer([set_RPI, a_RPI == vector],0,ops2,in,out);

% Optimizer MPC controllers
controller = cell(1,N);
for i = 0 : N-1
    horizon = N-i;
    [constraints,objective,x,u,G_in,c_in,A_in,b_in] = MPCFormulation(model,X,U,nx,nu,lqr,horizon,set_max);
    parameters_in = {x{1},G_in,c_in,A_in,b_in}; 
    if horizon == 1
        solutions_out = {u,[x{:}]};
    else 
        solutions_out = {[u{:}],[x{:}]};
    end
    mpc = optimizer(constraints,objective,ops,parameters_in,solutions_out);
    controller{i+1} = mpc;
end
disp('Optimizers computed.');
%% Run MPC for different vertices in the Region of Atraction
n_sim = 5; % number of Monte Carlo runs
j = 0; % count the number of simulations
v_min = 4.5; v_max = 2*v_min; % limits to search for a initial condition

while j < n_sim   
    vector = -v_min + v_max*rand(nx,1);
    % Find vertex with mode 1. or 2.
    % 1. Boundary of RA
    vector = vector./norm(vector); [vertex, ~] = boundary{vector};
    % 2. Inside RA
%     [vertex, ~] = confirmation{vector};
    
    % To consider multiple vertices
    if vertex ~= zeros(nx,1) 
        j = j+1;
        
        % Define the initial condition
        x0 = vertex;% + 3.05*vector;
      
        % Analyse of the initial condition
        [~, flag] = confirmation{x0};
        if flag ~= 0 
            disp('Not in RA.')
            subplot(1,3,1); plot(x0(1),x0(2),'*','LineWidth',1);
            subplot(1,3,2); plot(x0(3),x0(4),'*','LineWidth',1);
            subplot(1,3,3); plot(x0(5),x0(6),'*','LineWidth',1); 
        end
        
        % Find the larger set in which x0 belongs
%         idx = FindSet(Xj_list,x0);
%         idx = size(Xj_list,1);
        
        % Run the MPC with the initial condition x0
        path = MPCSimulation(controller,x0,Nc,model,Xj_list,N,terminal,idx,set_max);
               
        if n_dimensions == 2
            subplot(1,3,1);
            plot(path(1,:),path(2,:),'-o','LineWidth',2);
            subplot(1,3,2);
            plot(path(3,:),path(4,:),'-o','LineWidth',2);
            subplot(1,3,3);
            plot(path(5,:),path(6,:),'-o','LineWidth',2);
        elseif n_dimensions == 3
            x_axis = repmat(0 : size(path,2)-1, 1, 1);
            subplot(1,3,1);
            plot3(x_axis,path(1,:),path(2,:),'-o','LineWidth',2);
            subplot(1,3,2);
            plot3(x_axis,path(3,:),path(4,:),'-o','LineWidth',2);
            subplot(1,3,3);
            plot3(x_axis,path(5,:),path(6,:),'-o','LineWidth',2);  
        end
    end
end