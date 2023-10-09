function PlotSets(X,U,D,nx,nu)
% PlotSets - Function that plots each CCG in 3D
% 
% Syntax:  
%    PlotSets(X,U,D,nx,nu)
%
% Inputs:
%    X - Constrained Convex Generator of state constraints
%    U - Constrained Convex Generator of input constraints
%    U - Constrained Convex Generator of disturbances
%    nx - state dimensions
%    nu - input dimensions
% 
% Other m-files required: compileCCG.m
% Subfunctions: none
% MAT-files required: none 

%------------- BEGIN CODE --------------

% Plot X
% Figure setup
figure;
fontSize = 16;
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

% Compile & plot X
[set_X,a_X] = compileCCG(X);
subplot(1,3,1); plot(set_X,a_X(1:nu));
subplot(1,3,2); plot(set_X,a_X(nu+1:nu*2));
subplot(1,3,3); plot(set_X,a_X(nu*2+1:nx));

% ----------------------------------------------------------
% Plot U
% Figure setup
figure;
fontSize = 16;

xlabel('u_x [m/s^3]','FontSize',fontSize+2);
ylabel('u_y [m/s^3]','FontSize',fontSize+2); 
zlabel('u_z [m/s^3]','FontSize',fontSize+2); 
ax = gca(); ax.FontSize = fontSize;

% Compile & plot U
[set_U,a_U] = compileCCG(U);
plot(set_U,a_U);

% ----------------------------------------------------------
% Plot D
% Figure setup
figure;
fontSize = 16;
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

% Compile & plot D
[set_D,a_D] = compileCCG(D);
subplot(1,3,1); plot(set_D,a_D(1:nu));
subplot(1,3,2); plot(set_D,a_D(nu+1:nu*2));
subplot(1,3,3); plot(set_D,a_D(nu*2+1:nx));

% ----------------------------
% Figure setup
figure;
fontSize = 16;
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

% Compile & plot D
[set_D,a_D] = compileCCG(D);
subplot(1,3,1); plot(set_D,a_D(1:nu));
subplot(1,3,2); plot(set_D,a_D(nu+1:nu*2));
subplot(1,3,3); plot(set_D,a_D(nu*2+1:nx));

% ------------------------------------------------
% Plot RPI 
% Note: Run CCGInnerRPI 2 times in command window
% RPI = CCGInnerRPI(Acl, D, steps); % without last line
% RPI2 = CCGInnerRPI(Acl, D, steps); % with last line

% Figure setup
figure;
fontSize = 16;
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

% Compile & plot RPI
[set_RPI,a_RPI] = compileCCG(RPI);
[set_RPI2,a_RPI2] = compileCCG(RPI2);
subplot(1,3,1); 
plot(set_RPI2,a_RPI2(1:nu),'b'); 
plot(set_RPI,a_RPI(1:nu),'r'); 
subplot(1,3,2); 
plot(set_RPI2,a_RPI2(nu+1:nu*2),'b'); 
plot(set_RPI,a_RPI(nu+1:nu*2),'r');
subplot(1,3,3); 
plot(set_RPI2,a_RPI2(nu*2+1:nx),'b'); 
plot(set_RPI,a_RPI(nu*2+1:nx),'r'); 

legend('Overbounded RPI','RPI');
end

