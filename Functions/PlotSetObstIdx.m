function PlotSetObstIdx(Xj_list,Obs,idx,N)
% PlotSetObstIdx - Function that plots the set X and O from the sequence 
% considering the obstacle introduction at idx
% 
% Syntax:  
%    PlotSetObstIdx(Xj_list,Obs,idx,N)
%
% Inputs:
%    Xj_list - sequence of Constrained Convex Generator
%    Obs - Constrained Convex Generator representing the obstacle
%    idx - index in the sequence where obstacle is faced
%    N - prediction horizon
% 
% Other m-files required: compileCCG.m
% Subfunctions: none
% MAT-files required: none 

%------------- BEGIN CODE --------------

% Figure setup
figure;
fontSize = 16;
% position
subplot(1,3,1); hold on;
xlabel('p_x [m]','FontSize',fontSize+2);
ylabel('p_y [m]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Position','FontSize',fontSize+4);
% velocity
subplot(1,3,2); hold on;
xlabel('v_x [m/s]','FontSize',fontSize+2);
ylabel('v_y [m/s]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Velocity','FontSize',fontSize+4);
subplot(1,3,3); hold on;
xlabel('a_x [m/s^2]','FontSize',fontSize+2);
ylabel('a_y [m/s^2]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;
title('Acceleration','FontSize',fontSize+4);

i = length(Xj_list)-idx;
plotargs = {'r','b','y','c'};
arg_1 = 1;
arg_2 = 3;
if mod(i-1,N) == 0
    arg_1 = 2; 
    arg_2 = 4;
end
    
p = 1;
for a = 1 : 2 : 5
    b = a+1;
    
    X = Xj_list{i}; Y = Obs;
    Z = CCGIntersect2Sets(X,Y);
    [Fx,px] = compileCCG(X);
    [Fz,pz] = compileCCG(Z);
    subplot(1,3,p);
    plot(Fx,px(a:b,:),plotargs{arg_1});hold on;
    vertex = plot(Fz,pz(a:b,:));
    plot(vertex{1}(1,:),vertex{1}(2,:),plotargs{arg_2},'LineWidth',2);
    p = p+1;
end
lgd1 = strcat('X_',num2str(i-1));
lgd2 = strcat('O_',num2str(i-1));
legend(lgd1,lgd2);
end
