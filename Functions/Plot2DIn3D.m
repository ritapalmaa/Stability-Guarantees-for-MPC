function Plot2DIn3D(Xj_pre,Xj_list,nx,nu,N)
%PLOT2DIN3D Summary of this function goes here
%   Detailed explanation goes here
    figure;
    fontSize = 16;
    plotargs = {'r','b','off','on'};

    % position
    subplot(1,3,1); hold on;
    xlabel('k','FontSize',fontSize+2);
    ylabel('p_x','FontSize',fontSize+2); 
    zlabel('p_y','FontSize',fontSize+2);
    ax = gca(); ax.FontSize = fontSize;
    title('Position','FontSize',fontSize+4);

    % velocity
    subplot(1,3,2); hold on;
    xlabel('k','FontSize',fontSize+2);
    ylabel('v_x','FontSize',fontSize+2); 
    zlabel('v_y','FontSize',fontSize+2); 
    ax = gca(); ax.FontSize = fontSize;
    title('Velocity','FontSize',fontSize+4);

    % acceleration
    subplot(1,3,3); hold on;
    xlabel('k','FontSize',fontSize+2);
    ylabel('a_x','FontSize',fontSize+2); 
    zlabel('a_y','FontSize',fontSize+2);
    ax = gca(); ax.FontSize = fontSize;
    title('Acceleration','FontSize',fontSize+4);

    idx = 0;

    % Compile & plot pre-reduction set
    [set_Xj_pre,a_Xj_pre] = compileCCG(Xj_pre);

    first = plot(set_Xj_pre,a_Xj_pre(1:nu)); x_axis1 = idx * ones(1,size(first{1},2));
    second = plot(set_Xj_pre,a_Xj_pre(nu+1:nu*2)); x_axis2 = idx * ones(1,size(second{1},2));
    third = plot(set_Xj_pre,a_Xj_pre(nu*2+1:nx)); x_axis3 = idx * ones(1,size(third{1},2));
    
    subplot(1,3,1); plot3(x_axis1,first{1}(1,:),first{1}(2,:),'c','HandleVisibility','off');
    subplot(1,3,2); plot3(x_axis2,second{1}(1,:),second{1}(2,:),'c','HandleVisibility','off');
    subplot(1,3,3); plot3(x_axis3,third{1}(1,:),third{1}(2,:),'c','HandleVisibility','off');

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
        % prep
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
    
    legend('X_{j-h}, h = 1, ..., N_c','X_{i_{j-h}}, i = 1, ..., N', 'Xf');

end

