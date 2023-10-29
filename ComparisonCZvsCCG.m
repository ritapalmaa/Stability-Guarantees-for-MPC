close all;
%%
clc, clear;
yalmip('clear');
%%
addpath('CCGFunctions');
addpath('Functions');
n_dimensions = 3; % 2-dimensions or 3-dimensions (drone)
%% Comparison CZs vs CCGs
% Initialization
if n_dimensions == 2
    N = 4;
    Nc = 1; 
    simulation_steps = N*Nc-1;

    model.A = [2, 1; 0, 2]; model.B = [1, 0; 0, 1];
    nx = 2; nu = 2;

    % Sets
    X.G = 20*eye(nx); X.c = zeros(nx,1);
    X.A = zeros(0,nx); X.b = zeros(0,1); 
    X.type = Inf; X.idx = nx;

    U.G = 2*eye(nu); U.c = zeros(nu,1); 
    U.A = zeros(0,nu); U.b = zeros(0,1); 
    U.type = 2; U.idx = nu;

    RPI.G = 0.001*eye(nx); RPI.c = [0;0];
    RPI.A = zeros(0,nx); RPI.b = zeros(0,1); 
    RPI.type = 2; RPI.idx = nx;
    [set_RPI,a_RPI] = compileCCG(RPI); 

    % LQR Matrices
    lqr.Q = diag(ones(1,nx)); lqr.R = eye(nu);
    [lqr.K,lqr.P,~] = dlqr(model.A,model.B,lqr.Q,lqr.R);
    Acl = model.A - model.B * lqr.K;
else
    nx = 9; nu = 3; N = 4;
    Nc = 3; 
    simulation_steps = N*Nc-1;
    Ts = 0.1; %[s]
    [model,X,U,D,RPI,lqr,Acl] = InitDrone(nx,nu,Ts,simulation_steps);
end

% Compute Backward reachable sets with CZs and CCGs
Xj_list = cell(Nc*N+1,1); CZ_Xj_list = cell(Nc*N+1,1); 
Xj = RPI; CZ_Xj = RPI;

% Backward reachability sets
for i = 1 : (Nc*N) +1
    Xj_list{i} = Xj; CZ_Xj_list{i} = Xj;
    Xj_prev_est = CCGMinkowskiSum(CCGLinMap(model.A^-1,Xj,zeros(nx,1)),CCGLinMap(-(model.A^-1)*model.B,U,zeros(nx,1))); % est means estimate
    CZ_Xj_prev_est = CZMinkowskiSum(CZLinMap(model.A^-1,Xj,zeros(nx,1)),CZLinMap(-(model.A^-1)*model.B,U,zeros(nx,1))); 
    Xj = CCGIntersect(eye(nx),Xj_prev_est,X);
    CZ_Xj = CZIntersect(eye(nx),CZ_Xj_prev_est,X);
end

[set_Xj_pre,a_Xj_pre] = compileCCG(Xj_list{end}); % this set is used only for plots
[set_Xj,a_Xj] = compileCCG(Xj_list{end}); 
[CZ_set_Xj,CZ_a_Xj] = compileCZ(CZ_Xj_list{end});
disp('Region of attraction have been computed.');

% Plots
if n_dimensions == 2
    fontSize = 16;
    ax = gca(); ax.FontSize = fontSize;
    colors = ['c','m','b','r','w','g','k'];
    figure; hold on; 
    xlabel('x','FontSize',fontSize+2); 
    ylabel('y','FontSize',fontSize+2);
    idx = 1;
    % Plot of the Backward reachability sets in reverse order
    for i = flip(1 : length(Xj_list))    
        [set_Xj_plot,a_Xj_plot] = compileCZ(CZ_Xj_list{i}); 
        vertex = plot(set_Xj_plot,a_Xj_plot,colors(idx));
        [set_Xj_plot,a_Xj_plot] = compileCCG(Xj_list{i}); 
        plot(set_Xj_plot,a_Xj_plot,colors(idx)); 
        if i == 1
            plot(vertex{1}(1,:),vertex{1}(2,:),'k','LineWidth',2);
            plot(set_Xj_plot,a_Xj_plot,'k'); 
        else
            plot(vertex{1}(1,:),vertex{1}(2,:),'k','LineWidth',2,'HandleVisibility','Off');
        end
        idx = idx+1;
    end

    legend('X_4','X_3','X_2','X_1','X_f')
else
    figure; plotargs = {'r','b'}; fontSize = 16;

    subplot(1,3,1); hold on; 
    ax = gca(); ax.FontSize = fontSize;
    title('Position','FontSize',fontSize+4);
    xlabel('p_x [m]','FontSize',fontSize+2); 
    ylabel('p_y [m]','FontSize',fontSize+2);
    zlabel('p_z [m]','FontSize',fontSize+2);

    subplot(1,3,2); hold on; 
    ax = gca(); ax.FontSize = fontSize;
    title('Velocity','FontSize',fontSize+4);
    xlabel('v_x [m/s]','FontSize',fontSize+2); 
    ylabel('v_y [m/s]','FontSize',fontSize+2);
    zlabel('v_z [m/s]','FontSize',fontSize+2);

    subplot(1,3,3); hold on; 
    ax = gca(); ax.FontSize = fontSize;
    title('Acceleration','FontSize',fontSize+4);
    xlabel('a_x [m/s^2]','FontSize',fontSize+2); 
    ylabel('a_y [m/s^2]','FontSize',fontSize+2); 
    zlabel('a_z [m/s^2]','FontSize',fontSize+2);

    % Plot of the Backward reachability sets
    % for i = 1 : length(Xj_list) 
    i = length(Xj_list);
        if mod(i-1,N) == 0
            color = 2;
        else
            color = 1;
        end

        subplot(1,3,1);
        [set_Xj_plot,a_Xj_plot] = compileCZ(CZ_Xj_list{i}); 
        vertex = plot(set_Xj_plot,a_Xj_plot(1:nu),plotargs{color});
        plot3(vertex{1}(1,:),vertex{1}(2,:),vertex{1}(3,:),'k','LineWidth',2);
        [set_Xj_plot,a_Xj_plot] = compileCCG(Xj_list{i});
        plot(set_Xj_plot,a_Xj_plot(1:nu),plotargs{color});

        subplot(1,3,2);
        [set_Xj_plot,a_Xj_plot] = compileCZ(CZ_Xj_list{i}); 
        vertex = plot(set_Xj_plot,a_Xj_plot(nu+1:nu*2),plotargs{color});
        plot3(vertex{1}(1,:),vertex{1}(2,:),vertex{1}(3,:),'k','LineWidth',2);
        [set_Xj_plot,a_Xj_plot] = compileCCG(Xj_list{i});
        plot(set_Xj_plot,a_Xj_plot(nu+1:nu*2),plotargs{color});

        subplot(1,3,3);
        [set_Xj_plot,a_Xj_plot] = compileCZ(CZ_Xj_list{i}); 
        vertex = plot(set_Xj_plot,a_Xj_plot(nu*2+1:nx),plotargs{color});
        plot3(vertex{1}(1,:),vertex{1}(2,:),vertex{1}(3,:),'k','LineWidth',2);
        [set_Xj_plot,a_Xj_plot] = compileCCG(Xj_list{i});
        plot(set_Xj_plot,a_Xj_plot(nu*2+1:nx),plotargs{color});
    % end
    legend('CZ','CCG');
end