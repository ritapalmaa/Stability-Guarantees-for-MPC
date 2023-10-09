function [model,X,U,D,RPI,lqr,Acl] = InitDrone(nx,nu,Ts,steps)
% InitDrone - Function that initialize and set up the parameters in 3D
%
% Syntax:  
%    [model,X,U,D,RPI,lqr,Acl] = InitDrone(nx,nu,Ts,simulation_steps)
%
% Inputs:
%    nx - state dimensions
%    nu - input dimensions
%    Ts - sample time
%    steps - number of simulation steps
%
% Outputs:
%    model - struct with the system model A and B matrices
%    X - Constrained Convex Generator for state constraints
%    U - Constrained Convex Generator for input constraints
%    D - Constrained Convex Generator for disturbances
%    RPI - Constrained Convex Generator for RPI
%    lqr - struct that containts the lqr matrices (P,Q,R)
%    Acl - closed-loop matrix
%
% Other m-files required: CCGInnerRPI.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization -------------------------------------------------------
Tmax = 18; 
m = 1.375;  %[kg] 
max_p = [20; 20; 20]; %[m]
max_v = [10; 10; 6]; %[m/s]
% ------------------------------------------------------------------------
g = 9.8; %[m/s^2]
R = (Tmax/m);

% Discret Plant Model
model.A = [eye(nu) eye(nu)*Ts eye(nu)*(Ts^2/2);    
           zeros(nu) eye(nu) eye(nu)*Ts;
           zeros(nu) zeros(nu) eye(nu)];
model.B = [eye(nu)*Ts^3/6 eye(nu)*Ts^2/2  eye(nu)*Ts]';

% LQR Matrices
lqr.Q = diag([1000 1000 1000 100 100 100 10 10 10]);
lqr.R = 0.1*eye(nu);
[lqr.K,lqr.P,~] = dlqr(model.A,model.B,lqr.Q,lqr.R);
Acl = model.A - model.B * lqr.K;

% State Sets 
% Aceleration
A.G = eye(nu)*R; A.c = [0; 0; g];
A.A = zeros(0,nu); A.b = zeros(0,1); 
A.idx = nu; A.type = 2;
% rever ------------------------------------------------------------------
% Velocity
V.G = diag(max_v); V.c = zeros(nu,1);
V.A = zeros(0,nu); V.b = zeros(0,1);
V.idx = nu; V.type = Inf;
% Position
P.G = diag(max_p); P.c = zeros(nu,1);
P.A = zeros(0,nu); P.b = zeros(0,1);
P.idx = nu; P.type = Inf; 
% ------------------------------------------------------------------------

% Cartesian Sets
% Set B (Position & Velocity) - Cartesian Product
B = CCGCartesian(P,V);
% Set X (States)[pos vel ace] - Cartesian Product
X = CCGCartesian(B,A); 

% Input Set U % rever ----------------------------------------------------
a_min = 0; b_max = Tmax;
Ttil = a_min + 3;%(b_max-a_min)*rand();
omega = 2*pi; %random value

U.G = (Ttil/m)*omega*eye(nu); U.c = zeros(nu,1);
U.A = zeros(0,nu); U.b = zeros(0,1); 
U.idx = nu; U.type = 2;
% ------------------------------------------------------------------------

% Constraints Set G - by cartesian product
G = CCGCartesian(X,U);

% Disturbances Set D related with aceleration
D.G = [zeros(2*nu,nu); diag([1 1 1])];
D.c = zeros(nx,1);
D.A = zeros(0,nu); D.b = zeros(0,1);
D.idx = nu; D.type = Inf;

% Computing Terminal Set Xf - Feasible & Positive Invariant - by LQR
RPI = CCGInnerRPI(Acl, D, steps);
% using simple RPI by hand
% RPI.G = diag([0.05 0.05 0.01 0.01 0.01 0.1 0.10 0.01 0.01]); 
% RPI.c = zeros(nx,1); 
% RPI.A = zeros(0,nx); RPI.b = zeros(0,1); 
% RPI.type = Inf; RPI.idx = nx;
end

