function [model,X,U,D,RPI,lqr,Acl] = Init2D(nx,nu,Ts,steps)
% Init2D - Function that initialize and set up the parameters in 2D
%
% Syntax:  
%    [model,X,U,D,RPI,lqr,Acl] = Init2D(nx,nu,Ts,steps)
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

% Discret Plant Model
model.A = [eye(nu) eye(nu)*Ts eye(nu)*(Ts^2/2);    
           zeros(nu) eye(nu) eye(nu)*Ts;
           zeros(nu) zeros(nu) eye(nu)];
model.B = [eye(nu)*Ts^3/6 eye(nu)*Ts^2/2  eye(nu)*Ts]';

% LQR Matrices
lqr.Q = diag([1000 1000 100 100 10 10]);
lqr.R = 0.1*eye(nu);
[lqr.K,lqr.P,~] = dlqr(model.A,model.B,lqr.Q,lqr.R);
Acl = model.A - model.B * lqr.K;

% Sets
X.G = diag([10 10 7 7 13 13]); X.c = zeros(nx,1);
X.A = zeros(0,nx); X.b = zeros(0,1); 
X.type = Inf; X.idx = nx;

U.G = 10*eye(nu); U.c = zeros(nu,1);
U.A = zeros(0,nu); U.b = zeros(0,1); 
U.type = 2; U.idx = nu;

D.G = [zeros(2*nu,nu); eye(nu)];
D.c = zeros(nx,1);
D.A = zeros(0,nu); D.b = zeros(0,1);
D.type = Inf; D.idx = nu;

% Computing Terminal Set Xf - Feasible & Positive Invariant - by LQR
RPI = CCGInnerRPI(Acl, D, steps);
end