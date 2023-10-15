function Plot2DIn3D(Xj_pre,Xj_list,nx,nu,N)
% Plot2DIn3D - Function that plots the CCGs for 2D in 3D 
% with 3 subplots for position, velocity and acceleration
%
% Syntax:  
%    Plot2DIn3D(Xj_pre,Xj_list,nx,nu,N)
%
% Inputs:
%    Xj_pre - Constrained Convex Generator
%    Xj_list - sequence of Constrained Convex Generator
%    nx - state dimensions
%    nu - input dimensions
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
plotargs = {'r','b','off','on'};
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

% Compile & plot pre-reduction set
[set_Xj_pre,a_Xj_pre] = compileCCG(Xj_pre);

first = plot(set_Xj_pre,a_Xj_pre(1:nu)); x_axis1 = idx * ones(1,size(first{1},2));
second = plot(set_Xj_pre,a_Xj_pre(nu+1:nu*2)); x_axis2 = idx * ones(1,size(second{1},2));
third = plot(set_Xj_pre,a_Xj_pre(nu*2+1:nx)); x_axis3 = idx * ones(1,size(third{1},2));

% subplot(1,3,1); plot3(x_axis1,first{1}(1,:),first{1}(2,:),'c','HandleVisibility','off');
% subplot(1,3,2); plot3(x_axis2,second{1}(1,:),second{1}(2,:),'c','HandleVisibility','off');
% subplot(1,3,3); plot3(x_axis3,third{1}(1,:),third{1}(2,:),'c','HandleVisibility','off');

% Compile & plot sequence of sets
for i = flip(1 : length(Xj_list))
    arg_2 = 3;
    if i == 1 || i ==2
        arg_2 = 4;
    end
    arg_1 = 1;
    if mod(i-1,N) == 0
        arg_1 = 2;
    end
    % preparation
    [set_Xj_fake,a_Xj_fake] = compileCCG(Xj_list{i});
    first = plot(set_Xj_fake,a_Xj_fake(1:nu));
    x_axis1 = idx * ones(1,size(first{1},2));
    second = plot(set_Xj_fake,a_Xj_fake(nu+1:nu*2));
    x_axis2 = idx * ones(1,size(second{1},2));
    third = plot(set_Xj_fake,a_Xj_fake(nu*2+1:nx));
    x_axis3 = idx * ones(1,size(third{1},2));
    % plots
    subplot(1,3,1); plot3(x_axis1,first{1}(1,:),first{1}(2,:),plotargs{arg_1},'HandleVisibility',plotargs{arg_2});
    subplot(1,3,2); plot3(x_axis2,second{1}(1,:),second{1}(2,:),plotargs{arg_1},'HandleVisibility',plotargs{arg_2});
    subplot(1,3,3); plot3(x_axis3,third{1}(1,:),third{1}(2,:),plotargs{arg_1},'HandleVisibility',plotargs{arg_2});    

    idx = idx+1;
end

%Plot Xf in Xj
subplot(1,3,1);
x_axis1 = 0 * ones(1,size(first{1},2)); patch(x_axis1,first{1}(1,:),first{1}(2,:),plotargs{arg_1});
x_axis1 = (length(Xj_list)-1) * ones(1,size(first{1},2)); patch(x_axis1,first{1}(1,:),first{1}(2,:),plotargs{arg_1},'HandleVisibility','off');

subplot(1,3,2);
x_axis2 = 0 * ones(1,size(second{1},2)); patch(x_axis2,second{1}(1,:),second{1}(2,:),plotargs{arg_1});
x_axis2 = (length(Xj_list)-1) * ones(1,size(second{1},2)); patch(x_axis2,second{1}(1,:),second{1}(2,:),plotargs{arg_1},'HandleVisibility','off');

subplot(1,3,3);
x_axis3 = 0 * ones(1,size(third{1},2)); patch(x_axis3,third{1}(1,:),third{1}(2,:),plotargs{arg_1});
x_axis3 = (length(Xj_list)-1) * ones(1,size(third{1},2)); patch(x_axis3,third{1}(1,:),third{1}(2,:),plotargs{arg_1});

legend('X_{i}, i = 1, ..., N \times N_c', 'X_{N\times i}, i = 1, ..., N_c','X_f');

% Plot properties
% Set the rotation angles
azimuth = 33;   % Rotation angle in degrees around the z-axis
elevation = 15; % Elevation angle in degrees
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