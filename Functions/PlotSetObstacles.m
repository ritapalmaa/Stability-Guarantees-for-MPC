function PlotSetObstacles(Xj_list, Xj_list_obs,N)
% PlotSetObstacles - Function that plots the CCGs for 2D in 3D 
% with 3 subplots for position, velocity and acceleration
% overlapped with the previous CCGs computed
%
% Syntax:  
%    PlotSetObstacles(Xj_list, Xj_list_obs,N)
%
% Inputs:
%    Xj_list - sequence of Constrained Convex Generator without obstacle
%    Xj_list_obs - sequence of Constrained Convex Generator with obstacle
%    N - prediction horizon
% 
% Other m-files required: compileCCG.m
% Subfunctions: none
% MAT-files required: none 

%------------- BEGIN CODE --------------

% Initialization
idx = 0; % auxiliar variable for plots

% Figure setup
figure;
fontSize = 16;
plotargs = {'r','b','y','c'};
% position
subplot(1,3,1); hold on;
xlabel('k','FontSize',fontSize+2);
ylabel('p_x [m]','FontSize',fontSize+2); 
zlabel('p_y [m]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Position','FontSize',fontSize+4);
% velocity
subplot(1,3,2); hold on;
xlabel('k','FontSize',fontSize+2);
ylabel('v_x [m/s]','FontSize',fontSize+2); 
zlabel('v_y [m/s]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Velocity','FontSize',fontSize+4);
% acceleration
subplot(1,3,3); hold on;
xlabel('k','FontSize',fontSize+2);
ylabel('a_x [m/s^2]','FontSize',fontSize+2); 
zlabel('a_y [m/s^2]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Acceleration','FontSize',fontSize+4);

for i = flip(1 : length(Xj_list))
    arg_1 = 1;
    if mod(i-1,N) == 0
        arg_1 = 2;
    end
    % Sequence without obstacles
    [set_Xj_fake,a_Xj_fake] = compileCCG(Xj_list{i});
    % preparation
    first = plot(set_Xj_fake,a_Xj_fake(1:2)); x_axis1 = idx * ones(1,size(first{1},2));
    second = plot(set_Xj_fake,a_Xj_fake(3:4)); x_axis2 = idx * ones(1,size(second{1},2));
    third = plot(set_Xj_fake,a_Xj_fake(5:6)); x_axis3 = idx * ones(1,size(third{1},2));
    % plot
    subplot(1,3,1); patch(x_axis1,first{1}(1,:),first{1}(2,:),plotargs{arg_1},'FaceAlpha',0.3);
    subplot(1,3,2); patch(x_axis2,second{1}(1,:),second{1}(2,:),plotargs{arg_1},'FaceAlpha',0.3);
    subplot(1,3,3); patch(x_axis3,third{1}(1,:),third{1}(2,:),plotargs{arg_1},'FaceAlpha',0.3);
    
    arg_1 = 3;
    if mod(i-1,N) == 0
        arg_1 = 4;
    end
    % Sequence with obstacles
    [set_Xj_fake,a_Xj_fake] = compileCCG(Xj_list_obs{i});
    % preparation
    first = plot(set_Xj_fake,a_Xj_fake(1:2)); x_axis1 = idx * ones(1,size(first{1},2));
    second = plot(set_Xj_fake,a_Xj_fake(3:4)); x_axis2 = idx * ones(1,size(second{1},2));
    third = plot(set_Xj_fake,a_Xj_fake(5:6)); x_axis3 = idx * ones(1,size(third{1},2));
    % plot
    subplot(1,3,1); plot3(x_axis1,first{1}(1,:),first{1}(2,:),plotargs{arg_1},'LineWidth',2);
    subplot(1,3,2); plot3(x_axis2,second{1}(1,:),second{1}(2,:),plotargs{arg_1},'LineWidth',2);
    subplot(1,3,3); plot3(x_axis3,third{1}(1,:),third{1}(2,:),plotargs{arg_1},'LineWidth',2);    
    
    idx = idx+1;
end

% Plot properties
% Set the rotation angles
azimuth = 16.4;   % Rotation angle in degrees around the z-axis
elevation = 13.8; % Elevation angle in degrees
% Rotate all subplots
subplot(1, 3, 1); 
view(azimuth, elevation);
xlim([0 length(Xj_list)+1]);
box off;

subplot(1, 3, 2);
view(azimuth, elevation);
xlim([0 length(Xj_list)+1]);
box off;

subplot(1, 3, 3);
view(azimuth, elevation);
xlim([0 length(Xj_list)+1]);
box off;
end