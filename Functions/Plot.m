function Plot(Xj_pre,Xj_list,nx,nu,N)
%PLOT Summary of this function goes here
%   Detailed explanation goes here
    figure;
    fontSize = 16;
    plotargs = {'r','b'};

    % position
    subplot(1,3,1); hold on;
    xlabel('p_x','FontSize',fontSize+2);
    ylabel('p_y','FontSize',fontSize+2); 
    ax = gca(); ax.FontSize = fontSize;
    title('Position','FontSize',fontSize+4);

    % velocity
    subplot(1,3,2); hold on;
    xlabel('v_x','FontSize',fontSize+2);
    ylabel('v_y','FontSize',fontSize+2); 
    ax = gca(); ax.FontSize = fontSize;
    title('Velocity','FontSize',fontSize+4);

    % acceleration
    subplot(1,3,3); hold on;
    xlabel('a_x','FontSize',fontSize+2);
    ylabel('a_y','FontSize',fontSize+2); 
    ax = gca(); ax.FontSize = fontSize;
    title('Acceleration','FontSize',fontSize+4);

    % Compile & plot pre-reduction set
    [set_Xj_pre,a_Xj_pre] = compileCCG(Xj_pre);

    subplot(1,3,1); plot(set_Xj_pre,a_Xj_pre(1:nu),'c');
    subplot(1,3,2); plot(set_Xj_pre,a_Xj_pre(nu+1:nu*2),'c');
    subplot(1,3,3); plot(set_Xj_pre,a_Xj_pre(nu*2+1:nx),'c');

    % Compile & plot sequence of sets (including RPI)
    for i = flip(1 : length(Xj_list))
        if mod(i-1,N) == 0
            color = 2;
        else
            color = 1;
        end
        [set_Xj_fake,a_Xj_fake] = compileCCG(Xj_list{i});
        subplot(1,3,1);
        plot(set_Xj_fake,a_Xj_fake(1:nu),plotargs{color});
        subplot(1,3,2);
        plot(set_Xj_fake,a_Xj_fake(nu+1:nu*2),plotargs{color});
        subplot(1,3,3);
        plot(set_Xj_fake,a_Xj_fake(nu*2+1:nx),plotargs{color});
    end
end

