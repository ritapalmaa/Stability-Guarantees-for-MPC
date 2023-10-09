function PlotGeral(Xj_pre,Xj_list,nx,nu,N)
% PlotGeral - Function that plots the CCGs for 2D and 3D 
% with 3 subplots for position, velocity and acceleration
% 
% Syntax:  
%    PlotGeral(Xj_pre,Xj_list,nx,nu,N)
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

% Figure setup
figure;
fontSize = 16;
plotargs = {'r','b'};
% position
subplot(1,3,1); hold on;
xlabel('p_x [m]','FontSize',fontSize+2);
ylabel('p_y [m]','FontSize',fontSize+2); 
zlabel('p_z [m]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Position','FontSize',fontSize+4);
% velocity
subplot(1,3,2); hold on;
xlabel('v_x [m/s]','FontSize',fontSize+2);
ylabel('v_y [m/s]','FontSize',fontSize+2); 
zlabel('v_z [m/s]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Velocity','FontSize',fontSize+4);
% acceleration
subplot(1,3,3); hold on;
xlabel('a_x [m/s^2]','FontSize',fontSize+2);
ylabel('a_y [m/s^2]','FontSize',fontSize+2); 
zlabel('a_z [m/s^2]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Acceleration','FontSize',fontSize+4);

% Compile & plot pre-reduction set
[set_Xj_pre,a_Xj_pre] = compileCCG(Xj_pre);

% subplot(1,3,1); plot(set_Xj_pre,a_Xj_pre(1:nu),'c','HandleVisibility','off');
% subplot(1,3,2); plot(set_Xj_pre,a_Xj_pre(nu+1:nu*2),'c','HandleVisibility','off');
% subplot(1,3,3); plot(set_Xj_pre,a_Xj_pre(nu*2+1:nx),'c','HandleVisibility','off');

% Compile & plot sequence of sets (including RPI)
for i = flip(1 : length(Xj_list))
    arg_1 = 1;
    if mod(i-1,N) == 0
        arg_1 = 2;
    end
    [set_Xj_fake,a_Xj_fake] = compileCCG(Xj_list{i});
    subplot(1,3,1);
    plot(set_Xj_fake,a_Xj_fake(1:nu),plotargs{arg_1});
    subplot(1,3,2);
    plot(set_Xj_fake,a_Xj_fake(nu+1:nu*2),plotargs{arg_1});
    subplot(1,3,3);
    plot(set_Xj_fake,a_Xj_fake(nu*2+1:nx),plotargs{arg_1});
end
end

