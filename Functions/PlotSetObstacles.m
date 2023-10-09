function PlotSetObstacles(Xj_list_obs,N)
% PlotSetObstacles - Function that plots the CCGs for 2D in 3D 
% with 3 subplots for position, velocity and acceleration
% overlapped with the previous CCGs computed
%
% Syntax:  
%    PlotSetObstacles(Xj_list_obs,N)
%
% Inputs:
%    Xj_list_obs - sequence of Constrained Convex Generator
%    N - prediction horizon
% 
% Other m-files required: compileCCG.m
% Subfunctions: none
% MAT-files required: none 

%------------- BEGIN CODE --------------

idx = 0; % auxiliar variable for plots

% Figure setup
plotargs = {'m','c','on','off'};

% Compile & plot sequence of sets
for i = flip(1 : length(Xj_list_obs))
    visible = 4;
    if i == 2 || i == 1
        visible = 3;
    end
    color = 1;
    if mod(i-1,N) == 0
        color = 2;
    end
    % preparation
    [set_Xj_fake,a_Xj_fake] = compileCCG(Xj_list_obs{i});
    first = plot(set_Xj_fake,a_Xj_fake(1:2)); x_axis1 = idx * ones(1,size(first{1},2));
    second = plot(set_Xj_fake,a_Xj_fake(3:4)); x_axis2 = idx * ones(1,size(second{1},2));
    third = plot(set_Xj_fake,a_Xj_fake(5:6)); x_axis3 = idx * ones(1,size(third{1},2));
    % plots
    subplot(1,3,1); plot3(x_axis1,first{1}(1,:),first{1}(2,:),plotargs{color},'LineWidth',2,'HandleVisibility',plotargs{visible});
    subplot(1,3,2); plot3(x_axis2,second{1}(1,:),second{1}(2,:),plotargs{color},'LineWidth',2,'HandleVisibility',plotargs{visible});
    subplot(1,3,3); plot3(x_axis3,third{1}(1,:),third{1}(2,:),plotargs{color},'LineWidth',2,'HandleVisibility',plotargs{visible});    

    idx = idx+1;
end
end

